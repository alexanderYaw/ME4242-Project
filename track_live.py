#!/usr/bin/env python3
"""Live view of EMIO's green-marker tracking: camera windows plus printed positions (mm, EMIO frame).

Run with the emio-venv Python:  ~/emio-venv/bin/python track_live.py
Stop with Ctrl+C or by closing the camera windows.
"""
import time

from emioapi import EmioCamera

PRINT_EVERY_S = 0.2


def main():
    cam = EmioCamera(show=True, track_markers=True)
    if not cam.open():
        raise SystemExit("Could not open the camera (is another program, e.g. emioapi calibrate, using it?)")
    print("Tracking green markers. Ctrl+C to stop.")
    last = 0.0
    try:
        while cam.is_running:
            cam.update()
            now = time.monotonic()
            if now - last >= PRINT_EVERY_S:
                last = now
                pos = cam.trackers_pos
                text = "  ".join(f"#{i}: ({p[0]:7.1f}, {p[1]:7.1f}, {p[2]:7.1f})" for i, p in enumerate(pos))
                print(f"\r{len(pos)} marker(s)  {text or 'none detected'}" + " " * 20, end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        cam.close()


if __name__ == "__main__":
    main()
