#!/usr/bin/env python3
"""Check the EMIO camera calibration using the two green markers on the calibration plate.

Run with the emio-venv Python:  ~/emio-venv/bin/python check_camera_calibration.py [--show] [--frames N]
Expected positions (from Emio Labs' calibration scene): (32, -230, 32) and (-32, -230, -32) mm.
"""
import argparse
import math
import sys

from emioapi import EmioCamera

EXPECTED = [(32.0, -230.0, 32.0), (-32.0, -230.0, -32.0)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true", help="show the camera / mask windows")
    ap.add_argument("--frames", type=int, default=60, help="frames to average (default 60)")
    args = ap.parse_args()

    cam = EmioCamera(show=args.show, track_markers=True)
    if not cam.open():
        sys.exit("Could not open the camera (is another program using it?)")
    sums = [[0.0, 0.0, 0.0] for _ in EXPECTED]
    counts = [0] * len(EXPECTED)
    try:
        for _ in range(args.frames):
            cam.update()
            for p in cam.trackers_pos:
                # assign each tracked blob to the nearest expected marker
                k = min(range(len(EXPECTED)), key=lambda j: math.dist(p[:3], EXPECTED[j]))
                for a in range(3):
                    sums[k][a] += p[a]
                counts[k] += 1
    finally:
        cam.close()

    for k, exp in enumerate(EXPECTED):
        if counts[k] == 0:
            print(f"marker near {exp}: not seen")
            continue
        avg = [s / counts[k] for s in sums[k]]
        err = math.dist(avg, exp)
        print(f"marker near {exp}: measured ({avg[0]:.1f}, {avg[1]:.1f}, {avg[2]:.1f})  "
              f"error {err:.1f} mm  (dx {avg[0]-exp[0]:+.1f}, dy {avg[1]-exp[1]:+.1f}, dz {avg[2]-exp[2]:+.1f})  "
              f"[{counts[k]}/{args.frames} frames]")
    print("Emio Labs expects roughly 2-4 mm error per marker.")


if __name__ == "__main__":
    main()
