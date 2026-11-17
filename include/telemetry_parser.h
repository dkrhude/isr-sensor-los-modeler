/**
 * @file telemetry_parser.h
 * @brief Embedded NMEA-0183 GPS State Machine & 9-Axis IMU Jitter Filter
 *
 * Designed for deterministic execution without dynamic memory allocation (malloc)
 * across embedded MCUs (ESP32-S3) and desktop C11 compilers (MSVC / GCC / Clang).
 */

#ifndef TELEMETRY_PARSER_H
#define TELEMETRY_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NMEA_MAX_PAYLOAD_LEN 120

    /* MPU-9250 Nominal Datasheet Sensitivity Scale Factors */
#define MPU9250_ACCEL_LSB_PER_G     16384.0  /* +/- 2g full-scale range       */
#define MPU9250_GYRO_LSB_PER_DPS    131.0    /* +/- 250 deg/s full-scale      */
#define MPU9250_MAG_UT_PER_LSB      0.15     /* 16-bit AK8963 magnetometer    */
#define KNOTS_TO_MPS                0.514444

    typedef enum {
        NMEA_STATE_WAIT_SYNC = 0,
        NMEA_STATE_PAYLOAD,
        NMEA_STATE_CSUM_HI,
        NMEA_STATE_CSUM_LO
    } NmeaParserState_t;

    typedef struct {
        NmeaParserState_t state;
        char payload[NMEA_MAX_PAYLOAD_LEN];
        uint8_t payload_len;
        uint8_t calc_xor;
        char rx_csum_hex[3];
        uint32_t valid_sentences;
        uint32_t checksum_errors;
        uint32_t framing_errors;
    } NmeaStreamParser_t;

    typedef struct {
        double utc_seconds;
        double lat_wgs84_deg;
        double lon_wgs84_deg;
        double alt_msl_m;
        double ground_speed_mps;
        double course_true_deg;
        double hdop;
        uint8_t fix_quality;
        uint8_t satellites;
        bool fix_valid;
    } GpsFixData_t;

    typedef struct {
        int16_t accel_lsb[3]; /* X, Y, Z raw 16-bit signed accelerometer counts */
        int16_t gyro_lsb[3];  /* X, Y, Z raw 16-bit signed gyroscope counts     */
        int16_t mag_lsb[3];   /* X, Y, Z raw 16-bit signed magnetometer counts  */
    } ImuRaw9Axis_t;

    typedef struct {
        bool initialized;
        double alpha_comp;    /* Complementary filter gyro weight (e.g., 0.96)  */
        double beta_lpf;      /* First-order IIR vibration low-pass coefficient */
        double ax_lpf;
        double ay_lpf;
        double az_lpf;
        double gx_lpf;
        double gy_lpf;
        double gz_lpf;
        double roll_deg;
        double pitch_deg;
        double yaw_deg;
    } ImuFilterState_t;

    typedef struct {
        uint32_t frame_index;
        double timestamp_sec;
        double lat_deg;
        double lon_deg;
        double alt_msl_m;
        double speed_mps;
        double raw_accel_roll_deg;
        double raw_accel_pitch_deg;
        double filt_roll_deg;
        double filt_pitch_deg;
        double filt_yaw_deg;
        bool gps_valid;
    } KinematicStateFrame_t;

    void nmea_parser_init(NmeaStreamParser_t* parser);
    uint8_t nmea_compute_checksum(const char* payload_no_delims);
    bool nmea_feed_byte(NmeaStreamParser_t* parser, char ch, GpsFixData_t* gps_state);
    bool nmea_parse_line(NmeaStreamParser_t* parser, const char* sentence, GpsFixData_t* gps_state);

    void imu_filter_init(ImuFilterState_t* filt, double alpha_comp, double beta_lpf);
    void imu_process_sample(
        ImuFilterState_t* filt,
        const ImuRaw9Axis_t* raw,
        double dt_sec,
        double* out_raw_roll_deg,
        double* out_raw_pitch_deg
    );

    void build_kinematic_frame(
        uint32_t frame_idx,
        double timestamp_sec,
        const GpsFixData_t* gps,
        const ImuFilterState_t* imu,
        double raw_roll_deg,
        double raw_pitch_deg,
        KinematicStateFrame_t* out_frame
    );

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_PARSER_H */