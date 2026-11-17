/**
 * @file main_telemetry_demo.c
 * @brief Simulates 50 Hz MPU-9250 IMU vibration + 1 Hz NMEA-0183 UART stream (including
 *        corrupted UART packets), runs the C telemetry state machine, reports RMS jitter
 *        attenuation, and exports kinematic state frames to CSV.
 */

#define _CRT_SECURE_NO_WARNINGS
#include "telemetry_parser.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD (M_PI / 180.0)

 /* Deterministic LCG pseudo-random generator [-1.0, +1.0] for reproducible hardware tests */
static double rand_symmetric(uint32_t* seed) {
    *seed = (*seed * 1664525u) + 1013904223u;
    double unit = (double)(*seed & 0x00FFFFFFu) / (double)0x00FFFFFFu;
    return (unit * 2.0) - 1.0;
}

int main(void) {
    const int total_samples = 250; /* 5.0 seconds at 50 Hz IMU rate */
    const double dt = 0.02;
    const char* csv_path = "data/processed/c_telemetry_state_frames.csv";

    NmeaStreamParser_t nmea;
    GpsFixData_t gps = { 0 };
    ImuFilterState_t imu;

    nmea_parser_init(&nmea);
    imu_filter_init(&imu, 0.96, 0.25);

    FILE* fp = fopen(csv_path, "w");
    if (!fp) {
        fprintf(stderr, "[ERROR] Unable to open %s for writing. Ensure data/processed exists.\n", csv_path);
        return 1;
    }

    fprintf(fp,
        "frame_idx,timestamp_sec,lat_wgs84,lon_wgs84,alt_msl_m,speed_mps,"
        "true_roll_deg,raw_accel_roll_deg,filt_roll_deg,filt_pitch_deg,filt_yaw_deg\n"
    );

    uint32_t rng_seed = 42026u;
    double sum_sq_raw_err = 0.0;
    double sum_sq_filt_err = 0.0;

    printf("================================================================================\n");
    printf("EMBEDDED C11 TELEMETRY STATE MACHINE & IMU JITTER FILTER BENCHMARK\n");
    printf("================================================================================\n");

    for (int k = 0; k < total_samples; ++k) {
        double t = k * dt;

        /* Every 50 samples (1 Hz), feed $GPGGA and $GPRMC UART sentences */
        if (k % 50 == 0) {
            int sec = (int)t;
            double lat_dm = 3433.0720 + (t * 0.051); /* Moving North at ~95 m/s */
            double lon_dm = 11752.9260;

            char gga_body[100];
            snprintf(
                gga_body, sizeof(gga_body),
                "GPGGA,1420%02d.00,%.4f,N,%.4f,W,1,11,0.82,7050.0,M,-31.2,M,,",
                sec, lat_dm, lon_dm
            );
            uint8_t gga_cs = nmea_compute_checksum(gga_body);
            char gga_full[120];
            snprintf(gga_full, sizeof(gga_full), "$%s*%02X\r\n", gga_body, gga_cs);
            nmea_parse_line(&nmea, gga_full, &gps);

            char rmc_body[100];
            snprintf(
                rmc_body, sizeof(rmc_body),
                "GPRMC,1420%02d.00,A,%.4f,N,%.4f,W,184.7,000.0,171126,,,A",
                sec, lat_dm, lon_dm
            );
            uint8_t rmc_cs = nmea_compute_checksum(rmc_body);
            char rmc_full[120];
            snprintf(rmc_full, sizeof(rmc_full), "$%s*%02X\r\n", rmc_body, rmc_cs);
            nmea_parse_line(&nmea, rmc_full, &gps);

            /* Inject 1 corrupted NMEA checksum frame at t = 2.0s to verify fault rejection */
            if (k == 100) {
                const char* corrupt_uart = "$GPGGA,142002.00,3433.1740,N,11752.9260,W,1,11,0.82,7050.0,M,-31.2,M,,*FF\r\n";
                nmea_parse_line(&nmea, corrupt_uart, &gps);
            }
        }

        /* True kinematic attitude: coordinated 26 deg banked turn entry + 2.5 deg pitch */
        double true_roll_deg = 26.0 * sin(0.25 * M_PI * (t / 5.0));
        double true_roll_rate_dps = 26.0 * (0.25 * M_PI / 5.0) * cos(0.25 * M_PI * (t / 5.0));
        double true_pitch_deg = 2.5;
        double true_yaw_deg = 12.0 * t;

        double phi = true_roll_deg * DEG_TO_RAD;
        double theta = true_pitch_deg * DEG_TO_RAD;
        double psi = true_yaw_deg * DEG_TO_RAD;

        /* Synthesize 18 Hz structural airframe / turret vibration + wideband MEMS noise */
        double vib_g = 0.085 * sin(2.0 * M_PI * 18.0 * t) + 0.035 * rand_symmetric(&rng_seed);
        double gyro_vib_dps = 4.5 * cos(2.0 * M_PI * 18.0 * t) + 1.2 * rand_symmetric(&rng_seed);

        double ax_g = -sin(theta) + vib_g;
        double ay_g = cos(theta) * sin(phi) + vib_g;
        double az_g = cos(theta) * cos(phi) + 0.5 * vib_g;

        ImuRaw9Axis_t raw_imu;
        raw_imu.accel_lsb[0] = (int16_t)(ax_g * MPU9250_ACCEL_LSB_PER_G);
        raw_imu.accel_lsb[1] = (int16_t)(ay_g * MPU9250_ACCEL_LSB_PER_G);
        raw_imu.accel_lsb[2] = (int16_t)(az_g * MPU9250_ACCEL_LSB_PER_G);

        raw_imu.gyro_lsb[0] = (int16_t)((true_roll_rate_dps + gyro_vib_dps) * MPU9250_GYRO_LSB_PER_DPS);
        raw_imu.gyro_lsb[1] = (int16_t)((0.5 * gyro_vib_dps) * MPU9250_GYRO_LSB_PER_DPS);
        raw_imu.gyro_lsb[2] = (int16_t)((12.0 + 0.5 * gyro_vib_dps) * MPU9250_GYRO_LSB_PER_DPS);

        /* Earth magnetic field (~48 uT horizontal) rotated into sensor body frame */
        double mag_n = 24.0;
        double mag_e = 0.0;
        double mag_d = 42.0;
        double mx_ut = mag_n * cos(theta) * cos(psi) + mag_e * cos(theta) * sin(psi) - mag_d * sin(theta);
        double my_ut = mag_n * (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi))
            + mag_e * (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi))
            + mag_d * sin(phi) * cos(theta);
        double mz_ut = mag_n * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi))
            + mag_e * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi))
            + mag_d * cos(phi) * cos(theta);

        raw_imu.mag_lsb[0] = (int16_t)(mx_ut / MPU9250_MAG_UT_PER_LSB);
        raw_imu.mag_lsb[1] = (int16_t)(my_ut / MPU9250_MAG_UT_PER_LSB);
        raw_imu.mag_lsb[2] = (int16_t)(mz_ut / MPU9250_MAG_UT_PER_LSB);

        double raw_roll = 0.0, raw_pitch = 0.0;
        imu_process_sample(&imu, &raw_imu, dt, &raw_roll, &raw_pitch);

        KinematicStateFrame_t frame;
        build_kinematic_frame((uint32_t)k, t, &gps, &imu, raw_roll, raw_pitch, &frame);

        double raw_err = frame.raw_accel_roll_deg - true_roll_deg;
        double filt_err = frame.filt_roll_deg - true_roll_deg;
        sum_sq_raw_err += raw_err * raw_err;
        sum_sq_filt_err += filt_err * filt_err;

        fprintf(fp,
            "%u,%.3f,%.6f,%.6f,%.1f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            frame.frame_index, frame.timestamp_sec,
            frame.lat_deg, frame.lon_deg, frame.alt_msl_m, frame.speed_mps,
            true_roll_deg, frame.raw_accel_roll_deg,
            frame.filt_roll_deg, frame.filt_pitch_deg, frame.filt_yaw_deg
        );
    }

    fclose(fp);

    double rms_raw = sqrt(sum_sq_raw_err / (double)total_samples);
    double rms_filt = sqrt(sum_sq_filt_err / (double)total_samples);
    double atten_db = 20.0 * log10(rms_filt / rms_raw);

    printf("[+] NMEA-0183 UART Stream  : %u valid sentences parsed | %u corrupt checksums rejected\n",
        nmea.valid_sentences, nmea.checksum_errors);
    printf("[+] Last WGS84 Fix         : Lat %.5f, Lon %.5f | Alt %.1f m MSL | GS %.1f m/s\n",
        gps.lat_wgs84_deg, gps.lon_wgs84_deg, gps.alt_msl_m, gps.ground_speed_mps);
    printf("[+] Unfiltered Roll Jitter : %.3f deg RMS (18 Hz airframe vibration)\n", rms_raw);
    printf("[+] Filtered Roll Jitter   : %.3f deg RMS (%.1f dB vibration attenuation)\n", rms_filt, atten_db);
    printf("[+] Exported State Frames  : %s (%d frames)\n\n", csv_path, total_samples);

    return 0;
}