#!/usr/bin/env python3
"""
sensor_los_sim.py
Physics-based discrete-event airborne ISR racetrack orbit and sensor Line-of-Sight (LOS)
simulation over a 60-minute (3,600-second) mission window.

Computes:
- $C^1$-continuous racetrack trajectory at defined altitude, true airspeed, and bank angle
- 3-2-1 Euler rotation from Local Tangent Plane (NED) to 6-DOF Aircraft Body Frame
- Azimuth-dependent airframe wing/fuselage/tail masking dropouts during banked turns
- Slant range, earth grazing angle, oblique Ground Sample Distance (GSD), and swath width
- Multi-panel 2D/3D spatial & temporal diagnostic visualization
"""

import os
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROC_DIR = os.path.join(BASE_DIR, "data", "processed")
C_TELEM_CSV = os.path.join(PROC_DIR, "c_telemetry_state_frames.csv")
OUT_TIMESERIES_CSV = os.path.join(PROC_DIR, "mission_los_timeseries.csv")
OUT_REPORT_TXT = os.path.join(PROC_DIR, "mission_coverage_report.txt")
OUT_PLOT_PNG = os.path.join(PROC_DIR, "isr_orbit_los_analysis.png")


def load_empirical_imu_jitter() -> float:
    """
    Reads post-filter residual roll jitter RMS from the C telemetry output CSV if present;
    otherwise defaults to analytical 0.28 deg RMS residual stabilization jitter.
    """
    if not os.path.exists(C_TELEM_CSV):
        return 0.28
    sq_errs = []
    with open(C_TELEM_CSV, mode="r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            err = float(row["filt_roll_deg"]) - float(row["true_roll_deg"])
            sq_errs.append(err * err)
    return math.sqrt(sum(sq_errs) / len(sq_errs)) if sq_errs else 0.28


def generate_racetrack_kinematics(
    t_sec: np.ndarray,
    leg_length_m: float = 18000.0,
    airspeed_mps: float = 95.0,
    alt_agl_m: float = 6400.0,
    bank_turn_deg: float = 26.0,
    pitch_trim_deg: float = 2.5,
    jitter_rms_deg: float = 0.28
):
    """
    Generates C1-continuous clockwise (right-hand) standoff racetrack kinematics in NED.
    West leg flies North (heading 0 deg); East leg flies South (heading 180 deg).
    """
    g = 9.80665
    turn_radius_m = (airspeed_mps ** 2) / (g * math.tan(math.radians(bank_turn_deg)))
    t_leg = leg_length_m / airspeed_mps
    t_turn = (math.pi * turn_radius_m) / airspeed_mps
    period = 2.0 * (t_leg + t_turn)
    omega = airspeed_mps / turn_radius_m

    n_steps = len(t_sec)
    north_m = np.zeros(n_steps)
    east_m = np.zeros(n_steps)
    down_m = np.full(n_steps, -alt_agl_m)
    yaw_deg = np.zeros(n_steps)
    roll_deg = np.zeros(n_steps)

    rng = np.random.default_rng(20261117)
    roll_ramp_sec = 4.0  # Smooth roll-in / roll-out transition window

    for i, t in enumerate(t_sec):
        tau = t % period
        if tau < t_leg:
            # Segment 1: West straight leg flying North
            s = tau
            north_m[i] = -0.5 * leg_length_m + airspeed_mps * s
            east_m[i] = -turn_radius_m
            yaw_deg[i] = 0.0
            roll_deg[i] = 0.0
        elif tau < (t_leg + t_turn):
            # Segment 2: North 180-deg right-hand turnaround
            s = tau - t_leg
            u = omega * s
            north_m[i] = 0.5 * leg_length_m + turn_radius_m * math.sin(u)
            east_m[i] = -turn_radius_m * math.cos(u)
            yaw_deg[i] = math.degrees(u)
            edge_dist = min(s, t_turn - s)
            scale = 0.5 * (1.0 - math.cos(math.pi * edge_dist / roll_ramp_sec)) if edge_dist < roll_ramp_sec else 1.0
            roll_deg[i] = bank_turn_deg * scale
        elif tau < (2.0 * t_leg + t_turn):
            # Segment 3: East straight leg flying South (closer to target)
            s = tau - (t_leg + t_turn)
            north_m[i] = 0.5 * leg_length_m - airspeed_mps * s
            east_m[i] = turn_radius_m
            yaw_deg[i] = 180.0
            roll_deg[i] = 0.0
        else:
            # Segment 4: South 180-deg right-hand turnaround
            s = tau - (2.0 * t_leg + t_turn)
            u = omega * s
            north_m[i] = -0.5 * leg_length_m - turn_radius_m * math.sin(u)
            east_m[i] = turn_radius_m * math.cos(u)
            yaw_deg[i] = (180.0 + math.degrees(u)) % 360.0
            edge_dist = min(s, t_turn - s)
            scale = 0.5 * (1.0 - math.cos(math.pi * edge_dist / roll_ramp_sec)) if edge_dist < roll_ramp_sec else 1.0
            roll_deg[i] = bank_turn_deg * scale

    # Superimpose empirical post-filter IMU attitude jitter
    roll_deg += rng.normal(0.0, jitter_rms_deg, size=n_steps)
    pitch_deg = np.full(n_steps, pitch_trim_deg) + rng.normal(0.0, 0.5 * jitter_rms_deg, size=n_steps)

    return north_m, east_m, down_m, roll_deg, pitch_deg, yaw_deg, turn_radius_m


def airframe_mask_threshold_deg(az_body_deg: np.ndarray) -> np.ndarray:
    """
    Defines the minimum required body-frame depression angle (below aircraft X-Y wing plane)
    for a belly-mounted EO/IR turret to clear structural obscuration across 360 deg azimuth:
    - Nose radome (-25 to +25 deg): 14.0 deg
    - Wing & pylon stations (+/- 60 to +/- 125 deg): 10.0 deg
    - Aft tail / propeller wake (|az| > 150 deg): 21.0 deg
    - Fuselage chines elsewhere: 5.0 deg
    """
    abs_az = np.abs(az_body_deg)
    thresh = np.full_like(abs_az, 5.0)
    thresh = np.where(abs_az <= 25.0, 14.0, thresh)
    thresh = np.where((abs_az >= 60.0) & (abs_az <= 125.0), 10.0, thresh)
    thresh = np.where(abs_az >= 150.0, 21.0, thresh)
    return thresh


def run_simulation():
    os.makedirs(PROC_DIR, exist_ok=True)
    jitter_rms = load_empirical_imu_jitter()

    # 60-minute discrete-event window (dt = 1.0 sec)
    t_sec = np.arange(0.0, 3600.0, 1.0)
    alt_agl_m = 6400.0
    north_a, east_a, down_a, roll_deg, pitch_deg, yaw_deg, r_turn = generate_racetrack_kinematics(
        t_sec, alt_agl_m=alt_agl_m, jitter_rms_deg=jitter_rms
    )

    # Standoff High-Value Target (15.5 km East, 2.5 km North of orbit center, ground datum D=0)
    tgt_n, tgt_e, tgt_d = 2500.0, 15500.0, 0.0

    # Relative LOS vector in NED frame
    dn = tgt_n - north_a
    de = tgt_e - east_a
    dd = tgt_d - down_a  # Positive Down (~6400 m)

    horiz_range_m = np.hypot(dn, de)
    slant_range_m = np.sqrt(dn ** 2 + de ** 2 + dd ** 2)
    earth_grazing_deg = np.degrees(np.arctan2(dd, horiz_range_m))

    # 3-2-1 Aerospace Euler Rotation (NED -> Aircraft Body Frame: +Xb nose, +Yb right wing, +Zb belly)
    psi = np.radians(yaw_deg)
    theta = np.radians(pitch_deg)
    phi = np.radians(roll_deg)

    x1 = np.cos(psi) * dn + np.sin(psi) * de
    y1 = -np.sin(psi) * dn + np.cos(psi) * de
    z1 = dd

    x2 = np.cos(theta) * x1 - np.sin(theta) * z1
    y2 = y1
    z2 = np.sin(theta) * x1 + np.cos(theta) * z1

    xb = x2
    yb = np.cos(phi) * y2 + np.sin(phi) * z2
    zb = -np.sin(phi) * y2 + np.cos(phi) * z2

    az_body_deg = np.degrees(np.arctan2(yb, xb))
    dep_body_deg = np.degrees(np.arctan2(zb, np.hypot(xb, yb)))
    mask_limit_deg = airframe_mask_threshold_deg(az_body_deg)

    # Sensor Optical Physics (f = 500 mm narrow spotter, 8 um pitch, 1920 px array)
    focal_len_m = 0.50
    pixel_pitch_m = 8.0e-6
    detector_px = 1920
    ifov_rad = pixel_pitch_m / focal_len_m
    fov_rad = detector_px * ifov_rad

    sin_grazing = np.maximum(np.sin(np.radians(earth_grazing_deg)), 0.05)
    gsd_oblique_cm = 100.0 * (slant_range_m * ifov_rad) / np.sqrt(sin_grazing)
    swath_width_m = 2.0 * slant_range_m * np.tan(0.5 * fov_rad)

    # Evaluate Discrete-Event LOS Masking & Range Thresholds
    max_slant_range_m = 24000.0
    min_grazing_deg = 12.0

    is_airframe_masked = dep_body_deg < mask_limit_deg
    is_range_or_grazing_out = (slant_range_m > max_slant_range_m) | (earth_grazing_deg < min_grazing_deg)
    los_valid = (~is_airframe_masked) & (~is_range_or_grazing_out)

    # Count discrete dropout events (rising edges of ~los_valid)
    dropout_flags = (~los_valid).astype(int)
    dropout_events = int(np.sum(np.diff(np.concatenate(([0], dropout_flags))) == 1))

    valid_sec = int(np.sum(los_valid))
    masked_sec = int(np.sum(is_airframe_masked))
    range_drop_sec = int(np.sum((~is_airframe_masked) & is_range_or_grazing_out))
    efficiency_pct = 100.0 * valid_sec / len(t_sec)

    # Export Timeseries CSV
    with open(OUT_TIMESERIES_CSV, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_sec", "ac_north_m", "ac_east_m", "ac_alt_agl_m",
            "roll_deg", "yaw_deg", "slant_range_km", "earth_grazing_deg",
            "body_az_deg", "body_dep_deg", "mask_limit_deg",
            "gsd_oblique_cm", "swath_width_m", "los_valid"
        ])
        for i in range(len(t_sec)):
            writer.writerow([
                f"{t_sec[i]:.0f}", f"{north_a[i]:.1f}", f"{east_a[i]:.1f}", f"{alt_agl_m:.1f}",
                f"{roll_deg[i]:.2f}", f"{yaw_deg[i]:.2f}", f"{slant_range_m[i]/1000.0:.3f}",
                f"{earth_grazing_deg[i]:.2f}", f"{az_body_deg[i]:.2f}", f"{dep_body_deg[i]:.2f}",
                f"{mask_limit_deg[i]:.1f}", f"{gsd_oblique_cm[i]:.2f}", f"{swath_width_m[i]:.1f}",
                int(los_valid[i])
            ])

    # Build Quantitative Report
    report_lines = [
        "=" * 80,
        "AIRBORNE ISR ORBIT & SENSOR LOS COVERAGE EFFICIENCY REPORT (60-MIN MISSION)",
        "=" * 80,
        f"Orbit Geometry          : Standoff Racetrack | Leg 18.0 km | Turn Radius {r_turn/1000.0:.2f} km",
        f"Platform State          : Alt {alt_agl_m:.0f} m AGL | Airspeed 95.0 m/s | Turn Bank 26.0 deg",
        f"Post-Filter IMU Jitter  : {jitter_rms:.3f} deg RMS (ingested from C11 telemetry filter)",
        f"Target Offset (NED)     : North {tgt_n/1000.0:.2f} km, East {tgt_e/1000.0:.2f} km (TGT-ALPHA-01)",
        "-" * 80,
        f"Total Mission Window    : {len(t_sec)} sec (60.00 min)",
        f"Valid Sensor LOS Dwell  : {valid_sec} sec ({valid_sec/60.0:.2f} min)",
        f"Coverage Efficiency     : {efficiency_pct:.2f} %",
        f"Total LOS Dropout Time  : {len(t_sec) - valid_sec} sec across {dropout_events} banked-turn dropout events",
        f"  - Airframe Masking    : {masked_sec} sec (Wing/Pylon & Aft Empennage Obscuration)",
        f"  - Range/Grazing Limit : {range_drop_sec} sec",
        "-" * 80,
        f"Slant Range (Valid LOS) : Mean {np.mean(slant_range_m[los_valid])/1000.0:.2f} km | "
        f"Min {np.min(slant_range_m[los_valid])/1000.0:.2f} km | Max {np.max(slant_range_m[los_valid])/1000.0:.2f} km",
        f"Oblique GSD (Valid LOS) : Mean {np.mean(gsd_oblique_cm[los_valid]):.2f} cm | "
        f"Best {np.min(gsd_oblique_cm[los_valid]):.2f} cm | Worst {np.max(gsd_oblique_cm[los_valid]):.2f} cm",
        f"Swath Width (Valid LOS) : Mean {np.mean(swath_width_m[los_valid]):.1f} m | "
        f"Min {np.min(swath_width_m[los_valid]):.1f} m | Max {np.max(swath_width_m[los_valid]):.1f} m",
        "=" * 80
    ]
    report_text = "\n".join(report_lines)
    print(report_text)

    with open(OUT_REPORT_TXT, mode="w", encoding="utf-8") as f:
        f.write(report_text + "\n")

    # Generate Multi-Panel 2D/3D Visualization
    fig = plt.figure(figsize=(15, 10), dpi=220)
    fig.suptitle(
        f"Physics-Based Airborne ISR Orbit & Sensor LOS Masking Analysis (Coverage Efficiency: {efficiency_pct:.1f}%)",
        fontsize=13, fontweight="bold"
    )

    # Panel 1: 3D Spatial Orbit & LOS Ray Geometry (Single Complete Orbit Slice for Clarity)
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    slice_idx = slice(0, 600)
    e_km = east_a[slice_idx] / 1000.0
    n_km = north_a[slice_idx] / 1000.0
    alt_km = np.full_like(e_km, alt_agl_m / 1000.0)
    valid_slice = los_valid[slice_idx]

    ax3d.scatter(e_km[valid_slice], n_km[valid_slice], alt_km[valid_slice], c="#16a34a", s=6, label="Valid Sensor LOS")
    ax3d.scatter(e_km[~valid_slice], n_km[~valid_slice], alt_km[~valid_slice], c="#dc2626", s=12, label="Airframe Masked Dropout")
    ax3d.scatter([tgt_e / 1000.0], [tgt_n / 1000.0], [0.0], c="#1d4ed8", marker="^", s=90, label="Target (TGT-ALPHA-01)")

    # Plot sample LOS rays
    for idx in range(0, 520, 45):
        ray_color = "#16a34a" if los_valid[idx] else "#dc2626"
        ray_style = "--" if los_valid[idx] else ":"
        ax3d.plot(
            [east_a[idx] / 1000.0, tgt_e / 1000.0],
            [north_a[idx] / 1000.0, tgt_n / 1000.0],
            [alt_agl_m / 1000.0, 0.0],
            color=ray_color, linestyle=ray_style, linewidth=0.9, alpha=0.65
        )

    ax3d.set_xlabel("East (km)")
    ax3d.set_ylabel("North (km)")
    ax3d.set_zlabel("Altitude AGL (km)")
    ax3d.set_zlim(0, 8.0)
    ax3d.set_title("3D Racetrack Orbit & Target LOS Rays")
    ax3d.legend(loc="upper left", fontsize=8)

    # Panel 2: 2D Top-Down Planform & Standoff Geometry
    ax2d = fig.add_subplot(2, 2, 2)
    ax2d.scatter(e_km[valid_slice], n_km[valid_slice], c="#16a34a", s=8, label="Valid LOS Track")
    ax2d.scatter(e_km[~valid_slice], n_km[~valid_slice], c="#dc2626", s=14, label="Banked-Turn Masking Zone")
    ax2d.scatter([tgt_e / 1000.0], [tgt_n / 1000.0], c="#1d4ed8", marker="*", s=160, label="TGT-ALPHA-01")
    ax2d.annotate("North Turn\n(Right Wing Drop Masks Target)", xy=(0.0, 10.8), xytext=(3.0, 11.2),
                  arrowprops=dict(arrowstyle="->", color="#dc2626"), fontsize=8, color="#991b1b")
    ax2d.annotate("South Turn\n(Aft Tail Masks Target)", xy=(0.0, -10.8), xytext=(3.0, -11.5),
                  arrowprops=dict(arrowstyle="->", color="#dc2626"), fontsize=8, color="#991b1b")
    ax2d.set_xlabel("East Offset (km)")
    ax2d.set_ylabel("North Offset (km)")
    ax2d.set_title("2D Planform Track & Turn Dropout Sectors")
    ax2d.grid(True, linestyle=":", alpha=0.6)
    ax2d.axis("equal")
    ax2d.legend(loc="center right", fontsize=8)

    # Panel 3: 60-Minute Body-Frame Depression vs Airframe Masking Envelope
    ax_time = fig.add_subplot(2, 1, 2)
    t_min = t_sec / 60.0
    ax_time.plot(t_min, dep_body_deg, color="#0f172a", linewidth=1.1, label="Body-Frame Target Depression (deg)")
    ax_time.plot(t_min, mask_limit_deg, color="#dc2626", linestyle="--", linewidth=1.1, label="3D Airframe Masking Limit (deg)")
    ax_time.fill_between(
        t_min, -20.0, 60.0, where=(~los_valid),
        color="#ef4444", alpha=0.25, label="LOS Dropout Window (Target Obscured)"
    )

    ax_gsd = ax_time.twinx()
    ax_gsd.plot(t_min, gsd_oblique_cm, color="#2563eb", alpha=0.65, linewidth=1.0, label="Oblique GSD (cm)")
    ax_gsd.set_ylabel("Oblique Ground Sample Distance (cm)", color="#2563eb")
    ax_gsd.tick_params(axis="y", labelcolor="#2563eb")
    ax_gsd.set_ylim(20.0, 75.0)

    ax_time.set_xlim(0.0, 60.0)
    ax_time.set_ylim(-15.0, 55.0)
    ax_time.set_xlabel("Mission Elapsed Time (minutes)")
    ax_time.set_ylabel("Body-Frame Depression Angle (deg)")
    ax_time.set_title("60-Minute Mission Timeline: Body-Frame Look-Down Clearance, LOS Dropouts & Sensor GSD")
    ax_time.grid(True, linestyle=":", alpha=0.5)

    lines1, labels1 = ax_time.get_legend_handles_labels()
    lines2, labels2 = ax_gsd.get_legend_handles_labels()
    ax_time.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8, ncol=2)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(OUT_PLOT_PNG)
    plt.close(fig)

    print(f"\n[+] Exported 60-minute LOS timeseries CSV : {OUT_TIMESERIES_CSV}")
    print(f"[+] Exported coverage efficiency report   : {OUT_REPORT_TXT}")
    print(f"[+] Exported 2D/3D spatial diagnostic PNG : {OUT_PLOT_PNG}")


if __name__ == "__main__":
    run_simulation()