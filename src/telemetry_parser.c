/**
 * @file telemetry_parser.c
 * @brief Byte-stream NMEA-0183 parser ($GPGGA, $GPRMC) and 9-axis IMU vibration filter.
 */

#define _CRT_SECURE_NO_WARNINGS
#include "telemetry_parser.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

 /* Splits comma-delimited NMEA payload in-place while preserving empty (null) fields */
static int split_nmea_fields(char* buf, char** fields, int max_fields) {
    int count = 0;
    if (!buf || max_fields <= 0) {
        return 0;
    }
    fields[count++] = buf;
    for (char* p = buf; *p != '\0' && count < max_fields; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

/* Converts NMEA ddmm.mmmm or dddmm.mmmm string + hemisphere into WGS84 Decimal Degrees */
static bool parse_nmea_coord(const char* val_str, const char* hem_str, double* out_deg) {
    if (!val_str || !hem_str || val_str[0] == '\0' || hem_str[0] == '\0') {
        return false;
    }
    double raw = atof(val_str);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - ((double)degrees * 100.0);
    if (minutes < 0.0 || minutes >= 60.0) {
        return false;
    }
    double dd = (double)degrees + (minutes / 60.0);
    char h = (char)toupper((unsigned char)hem_str[0]);
    if (h == 'S' || h == 'W') {
        dd = -dd;
    }
    else if (h != 'N' && h != 'E') {
        return false;
    }
    *out_deg = dd;
    return true;
}

/* Converts NMEA HHMMSS.SS UTC string into seconds since midnight */
static double parse_utc_seconds(const char* utc_str) {
    if (!utc_str || strlen(utc_str) < 6) {
        return 0.0;
    }
    double val = atof(utc_str);
    int hh = (int)(val / 10000.0);
    int mm = (int)((val - hh * 10000.0) / 100.0);
    double ss = val - hh * 10000.0 - mm * 100.0;
    return (double)(hh * 3600 + mm * 60) + ss;
}

static bool dispatch_nmea_payload(const char* payload, GpsFixData_t* gps) {
    char scratch[NMEA_MAX_PAYLOAD_LEN];
    char* fields[20];

    strncpy(scratch, payload, sizeof(scratch) - 1);
    scratch[sizeof(scratch) - 1] = '\0';

    int n = split_nmea_fields(scratch, fields, 20);
    if (n < 10) {
        return false;
    }

    /* Match $GPGGA or multi-constellation $GNGGA */
    if (strcmp(fields[0], "GPGGA") == 0 || strcmp(fields[0], "GNGGA") == 0) {
        if (n < 10) {
            return false;
        }
        uint8_t fix_q = (uint8_t)atoi(fields[6]);
        gps->fix_quality = fix_q;
        gps->fix_valid = (fix_q > 0);
        gps->utc_seconds = parse_utc_seconds(fields[1]);
        gps->satellites = (uint8_t)atoi(fields[7]);
        if (fields[8][0] != '\0') {
            gps->hdop = atof(fields[8]);
        }
        if (fields[9][0] != '\0') {
            gps->alt_msl_m = atof(fields[9]);
        }
        double lat = 0.0, lon = 0.0;
        if (parse_nmea_coord(fields[2], fields[3], &lat) &&
            parse_nmea_coord(fields[4], fields[5], &lon)) {
            gps->lat_wgs84_deg = lat;
            gps->lon_wgs84_deg = lon;
        }
        return true;
    }

    /* Match $GPRMC or multi-constellation $GNRMC */
    if (strcmp(fields[0], "GPRMC") == 0 || strcmp(fields[0], "GNRMC") == 0) {
        if (n < 9) {
            return false;
        }
        char status = (char)toupper((unsigned char)fields[2][0]);
        gps->fix_valid = (status == 'A');
        gps->utc_seconds = parse_utc_seconds(fields[1]);
        double lat = 0.0, lon = 0.0;
        if (parse_nmea_coord(fields[3], fields[4], &lat) &&
            parse_nmea_coord(fields[5], fields[6], &lon)) {
            gps->lat_wgs84_deg = lat;
            gps->lon_wgs84_deg = lon;
        }
        if (fields[7][0] != '\0') {
            gps->ground_speed_mps = atof(fields[7]) * KNOTS_TO_MPS;
        }
        if (fields[8][0] != '\0') {
            gps->course_true_deg = atof(fields[8]);
        }
        return true;
    }

    return false;
}

void nmea_parser_init(NmeaStreamParser_t* parser) {
    if (!parser) {
        return;
    }
    memset(parser, 0, sizeof(NmeaStreamParser_t));
    parser->state = NMEA_STATE_WAIT_SYNC;
}

uint8_t nmea_compute_checksum(const char* payload_no_delims) {
    uint8_t xsum = 0;
    if (!payload_no_delims) {
        return 0;
    }
    for (const char* p = payload_no_delims; *p != '\0'; ++p) {
        xsum ^= (uint8_t)(*p);
    }
    return xsum;
}

bool nmea_feed_byte(NmeaStreamParser_t* parser, char ch, GpsFixData_t* gps_state) {
    if (!parser || !gps_state) {
        return false;
    }

    if (ch == '$') {
        parser->state = NMEA_STATE_PAYLOAD;
        parser->payload_len = 0;
        parser->calc_xor = 0;
        return false;
    }

    switch (parser->state) {
    case NMEA_STATE_WAIT_SYNC:
        break;

    case NMEA_STATE_PAYLOAD:
        if (ch == '*') {
            parser->payload[parser->payload_len] = '\0';
            parser->state = NMEA_STATE_CSUM_HI;
        }
        else if (ch == '\r' || ch == '\n') {
            parser->framing_errors++;
            parser->state = NMEA_STATE_WAIT_SYNC;
        }
        else if (parser->payload_len < (NMEA_MAX_PAYLOAD_LEN - 1)) {
            parser->payload[parser->payload_len++] = ch;
            parser->calc_xor ^= (uint8_t)ch;
        }
        else {
            parser->framing_errors++;
            parser->state = NMEA_STATE_WAIT_SYNC;
        }
        break;

    case NMEA_STATE_CSUM_HI:
        if (isxdigit((unsigned char)ch)) {
            parser->rx_csum_hex[0] = ch;
            parser->state = NMEA_STATE_CSUM_LO;
        }
        else {
            parser->framing_errors++;
            parser->state = NMEA_STATE_WAIT_SYNC;
        }
        break;

    case NMEA_STATE_CSUM_LO:
        parser->state = NMEA_STATE_WAIT_SYNC;
        if (!isxdigit((unsigned char)ch)) {
            parser->framing_errors++;
            return false;
        }
        parser->rx_csum_hex[1] = ch;
        parser->rx_csum_hex[2] = '\0';

        uint8_t rx_xor = (uint8_t)strtoul(parser->rx_csum_hex, NULL, 16);
        if (rx_xor != parser->calc_xor) {
            parser->checksum_errors++;
            return false;
        }

        if (dispatch_nmea_payload(parser->payload, gps_state)) {
            parser->valid_sentences++;
            return true;
        }
        break;
    }
    return false;
}

bool nmea_parse_line(NmeaStreamParser_t* parser, const char* sentence, GpsFixData_t* gps_state) {
    bool updated = false;
    if (!sentence) {
        return false;
    }
    for (const char* p = sentence; *p != '\0'; ++p) {
        if (nmea_feed_byte(parser, *p, gps_state)) {
            updated = true;
        }
    }
    return updated;
}

void imu_filter_init(ImuFilterState_t* filt, double alpha_comp, double beta_lpf) {
    if (!filt) {
        return;
    }
    memset(filt, 0, sizeof(ImuFilterState_t));
    filt->alpha_comp = alpha_comp;
    filt->beta_lpf = beta_lpf;
}

void imu_process_sample(
    ImuFilterState_t* filt,
    const ImuRaw9Axis_t* raw,
    double dt_sec,
    double* out_raw_roll_deg,
    double* out_raw_pitch_deg
) {
    /* 1. Convert 16-bit ADC counts into physical engineering units */
    double ax = (double)raw->accel_lsb[0] / MPU9250_ACCEL_LSB_PER_G;
    double ay = (double)raw->accel_lsb[1] / MPU9250_ACCEL_LSB_PER_G;
    double az = (double)raw->accel_lsb[2] / MPU9250_ACCEL_LSB_PER_G;

    double gx = (double)raw->gyro_lsb[0] / MPU9250_GYRO_LSB_PER_DPS;
    double gy = (double)raw->gyro_lsb[1] / MPU9250_GYRO_LSB_PER_DPS;
    double gz = (double)raw->gyro_lsb[2] / MPU9250_GYRO_LSB_PER_DPS;

    double mx = (double)raw->mag_lsb[0] * MPU9250_MAG_UT_PER_LSB;
    double my = (double)raw->mag_lsb[1] * MPU9250_MAG_UT_PER_LSB;
    double mz = (double)raw->mag_lsb[2] * MPU9250_MAG_UT_PER_LSB;

    /* Unfiltered accelerometer attitude (exhibits severe airframe vibration noise) */
    double raw_roll = atan2(ay, az) * RAD_TO_DEG;
    double raw_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    if (out_raw_roll_deg) *out_raw_roll_deg = raw_roll;
    if (out_raw_pitch_deg) *out_raw_pitch_deg = raw_pitch;

    /* 2. First-order IIR low-pass filter to attenuate high-frequency engine/gimbal jitter */
    if (!filt->initialized) {
        filt->ax_lpf = ax;
        filt->ay_lpf = ay;
        filt->az_lpf = az;
        filt->gx_lpf = gx;
        filt->gy_lpf = gy;
        filt->gz_lpf = gz;
        filt->roll_deg = raw_roll;
        filt->pitch_deg = raw_pitch;
        filt->yaw_deg = atan2(-my, mx) * RAD_TO_DEG;
        if (filt->yaw_deg < 0.0) filt->yaw_deg += 360.0;
        filt->initialized = true;
        return;
    }

    double b = filt->beta_lpf;
    filt->ax_lpf = b * ax + (1.0 - b) * filt->ax_lpf;
    filt->ay_lpf = b * ay + (1.0 - b) * filt->ay_lpf;
    filt->az_lpf = b * az + (1.0 - b) * filt->az_lpf;

    filt->gx_lpf = b * gx + (1.0 - b) * filt->gx_lpf;
    filt->gy_lpf = b * gy + (1.0 - b) * filt->gy_lpf;
    filt->gz_lpf = b * gz + (1.0 - b) * filt->gz_lpf;

    /* 3. Complementary sensor fusion (high-pass gyro integration + low-pass accel tilt) */
    double lpf_accel_roll = atan2(filt->ay_lpf, filt->az_lpf) * RAD_TO_DEG;
    double lpf_accel_pitch = atan2(
        -filt->ax_lpf,
        sqrt(filt->ay_lpf * filt->ay_lpf + filt->az_lpf * filt->az_lpf)
    ) * RAD_TO_DEG;

    double a = filt->alpha_comp;
    filt->roll_deg = a * (filt->roll_deg + filt->gx_lpf * dt_sec) + (1.0 - a) * lpf_accel_roll;
    filt->pitch_deg = a * (filt->pitch_deg + filt->gy_lpf * dt_sec) + (1.0 - a) * lpf_accel_pitch;

    /* 4. Tilt-compensated magnetometer heading fused with yaw-rate gyro */
    double phi = filt->roll_deg * DEG_TO_RAD;
    double theta = filt->pitch_deg * DEG_TO_RAD;
    double mag_x_h = mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi);
    double mag_y_h = my * cos(phi) - mz * sin(phi);
    double mag_yaw = atan2(-mag_y_h, mag_x_h) * RAD_TO_DEG;
    if (mag_yaw < 0.0) mag_yaw += 360.0;

    double gyro_yaw = filt->yaw_deg + filt->gz_lpf * dt_sec;
    while (gyro_yaw < 0.0) gyro_yaw += 360.0;
    while (gyro_yaw >= 360.0) gyro_yaw -= 360.0;

    /* Shortest-angle wraparound blend */
    double diff = mag_yaw - gyro_yaw;
    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;
    filt->yaw_deg = gyro_yaw + (1.0 - a) * diff;
    if (filt->yaw_deg < 0.0) filt->yaw_deg += 360.0;
    if (filt->yaw_deg >= 360.0) filt->yaw_deg -= 360.0;
}

void build_kinematic_frame(
    uint32_t frame_idx,
    double timestamp_sec,
    const GpsFixData_t* gps,
    const ImuFilterState_t* imu,
    double raw_roll_deg,
    double raw_pitch_deg,
    KinematicStateFrame_t* out_frame
) {
    if (!out_frame) return;
    out_frame->frame_index = frame_idx;
    out_frame->timestamp_sec = timestamp_sec;
    out_frame->lat_deg = gps->lat_wgs84_deg;
    out_frame->lon_deg = gps->lon_wgs84_deg;
    out_frame->alt_msl_m = gps->alt_msl_m;
    out_frame->speed_mps = gps->ground_speed_mps;
    out_frame->raw_accel_roll_deg = raw_roll_deg;
    out_frame->raw_accel_pitch_deg = raw_pitch_deg;
    out_frame->filt_roll_deg = imu->roll_deg;
    out_frame->filt_pitch_deg = imu->pitch_deg;
    out_frame->filt_yaw_deg = imu->yaw_deg;
    out_frame->gps_valid = gps->fix_valid;
}