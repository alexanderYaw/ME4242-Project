#!/usr/bin/env python3
"""On-chip calibration for an Intel RealSense D405, with before/after checking.

Usage:
    python d405_calibrate.py                  # aim -> measure -> calibrate -> compare -> ask to save
    python d405_calibrate.py --measure        # only measure current depth quality (changes nothing)
    python d405_calibrate.py --restore FILE   # write a backed-up calibration table back to the camera
    python d405_calibrate.py --factory-reset  # restore the factory calibration

Point the camera at a flat, well-lit, visually busy target (printed random pattern,
newspaper) that fills the view, roughly face-on, 30-50 cm away.
"""
import argparse
import select
import sys
import time

import numpy as np
import pyrealsense2 as rs

# Stream used for measuring depth quality (works over USB 2).
MEASURE_W, MEASURE_H, MEASURE_FPS = 640, 480, 30
# On-chip calibration requires this exact depth stream.
OCC_W, OCC_H, OCC_FPS = 256, 144, 90
# Central fraction of the image used for the plane fit.
ROI_FRACTION = 0.5
# Intel's health-check thresholds for on-chip calibration.
HEALTH_GOOD, HEALTH_OK = 0.25, 0.75


def get_device():
    devs = rs.context().query_devices()
    if len(devs) == 0:
        sys.exit("No RealSense device found.")
    dev = devs[0]
    info = lambda k: dev.get_info(k) if dev.supports(k) else "n/a"
    print(f"Camera: {info(rs.camera_info.name)}  serial {info(rs.camera_info.serial_number)}  "
          f"fw {info(rs.camera_info.firmware_version)}  USB {info(rs.camera_info.usb_type_descriptor)}")
    if not dev.is_auto_calibrated_device():
        sys.exit("This device does not support on-chip calibration.")
    return dev, dev.as_auto_calibrated_device()


def start_depth(serial, w, h, fps):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    profile = pipe.start(cfg)
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    for _ in range(30):  # let auto-exposure settle
        pipe.wait_for_frames()
    return pipe, scale, intr


def roi_points(frame, scale, intr):
    """Return (Nx3 points in metres from the central ROI, fill rate of the ROI)."""
    z = np.asanyarray(frame.get_data()).astype(np.float64) * scale
    h, w = z.shape
    mh, mw = int(h * (1 - ROI_FRACTION) / 2), int(w * (1 - ROI_FRACTION) / 2)
    z = z[mh:h - mh, mw:w - mw]
    v, u = np.mgrid[mh:h - mh, mw:w - mw]
    valid = z > 0
    fill = valid.mean()
    z, u, v = z[valid], u[valid], v[valid]
    x = (u - intr.ppx) / intr.fx * z
    y = (v - intr.ppy) / intr.fy * z
    return np.column_stack([x, y, z]), fill


def plane_rms(points):
    """RMS distance (m) of points from their best-fit plane."""
    centred = points - points.mean(axis=0)
    normal = np.linalg.svd(centred, full_matrices=False)[2][-1]
    return np.sqrt(np.mean((centred @ normal) ** 2))


def aim(serial):
    """Live readout until the user presses Enter."""
    pipe, scale, intr = start_depth(serial, MEASURE_W, MEASURE_H, MEASURE_FPS)
    print("\nAim the camera at the target. Aim for fill > 90%. Press Enter when ready.")
    try:
        while True:
            pts, fill = roi_points(pipe.wait_for_frames().get_depth_frame(), scale, intr)
            dist = np.median(pts[:, 2]) * 100 if len(pts) else 0
            print(f"\r  centre distance {dist:5.1f} cm   fill {fill * 100:5.1f}%   ", end="", flush=True)
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break
    finally:
        pipe.stop()
        print()


def measure(serial, label, n_frames=30):
    pipe, scale, intr = start_depth(serial, MEASURE_W, MEASURE_H, MEASURE_FPS)
    rms, fills, dists = [], [], []
    try:
        for _ in range(n_frames):
            pts, fill = roi_points(pipe.wait_for_frames().get_depth_frame(), scale, intr)
            fills.append(fill)
            if len(pts) > 100:
                rms.append(plane_rms(pts))
                dists.append(np.median(pts[:, 2]))
    finally:
        pipe.stop()
    if not rms:
        print(f"  [{label}] no valid depth in the centre of the image - check the target.")
        return None
    r, d = np.median(rms), np.median(dists)
    print(f"  [{label}] plane-fit RMS {r * 1000:.2f} mm ({r / d * 100:.2f}% of distance)   "
          f"fill {np.mean(fills) * 100:.1f}%   distance {d * 100:.1f} cm")
    return r


def save_table(table, path):
    with open(path, "wb") as f:
        f.write(bytes(table))
    print(f"  saved calibration table to {path}")


def run_occ(serial, acd, speed):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth, OCC_W, OCC_H, rs.format.z16, OCC_FPS)
    pipe.start(cfg)
    try:
        for _ in range(30):
            pipe.wait_for_frames()
        params = f'{{"calib type": 0, "speed": {speed}, "scan parameter": 0, "data sampling": 0}}'
        progress = lambda p: print(f"\r  progress {p:5.1f}%", end="", flush=True)
        table, health = acd.run_on_chip_calibration(params, progress, 30000)
        print()
        return table, health[0]
    finally:
        pipe.stop()


def describe_health(h):
    a = abs(h)
    if a < HEALTH_GOOD:
        return "GOOD - current calibration is fine; saving the new one is optional"
    if a < HEALTH_OK:
        return "FAIR - calibration can be improved"
    return "POOR - calibration needed"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--measure", action="store_true", help="only measure depth quality")
    ap.add_argument("--speed", type=int, default=2, choices=range(4),
                    help="calibration speed: 0 very fast, 1 fast, 2 medium (default), 3 slow/most thorough")
    ap.add_argument("--restore", metavar="FILE", help="write a backed-up calibration table to the camera")
    ap.add_argument("--factory-reset", action="store_true", help="restore factory calibration")
    args = ap.parse_args()

    dev, acd = get_device()
    serial = dev.get_info(rs.camera_info.serial_number)

    if args.factory_reset:
        acd.reset_to_factory_calibration()
        print("Factory calibration restored.")
        return
    if args.restore:
        with open(args.restore, "rb") as f:
            acd.set_calibration_table(list(f.read()))
        acd.write_calibration()
        print(f"Calibration from {args.restore} written to camera.")
        return

    aim(serial)
    print("Measuring current depth quality...")
    before = measure(serial, "current")
    if args.measure:
        return

    original = acd.get_calibration_table()
    save_table(original, f"calib_backup_{serial}_{time.strftime('%Y%m%d_%H%M%S')}.bin")

    print("Running on-chip calibration (keep the camera and target still)...")
    try:
        new_table, health = run_occ(serial, acd, args.speed)
    except RuntimeError as e:
        sys.exit(f"\nCalibration failed: {e}\nCheck the target is textured, flat, well lit and fills the view.")
    print(f"  health-check score {health:+.3f}  ->  {describe_health(health)}")

    acd.set_calibration_table(new_table)  # temporary: active until power cycle, not saved
    print("Measuring with the new calibration (not saved yet)...")
    after = measure(serial, "new")
    if before and after:
        print(f"  RMS change: {(after - before) / before * 100:+.1f}%  (negative = less noise = better)")

    if input("\nWrite the new calibration to the camera permanently? [y/N] ").strip().lower() == "y":
        acd.write_calibration()
        save_table(new_table, f"calib_new_{serial}_{time.strftime('%Y%m%d_%H%M%S')}.bin")
        print("New calibration written.")
    else:
        acd.set_calibration_table(original)
        print("Discarded; original calibration is active again.")


if __name__ == "__main__":
    main()
