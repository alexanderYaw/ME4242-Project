#!/usr/bin/env python3
"""Stream the tip position (green marker, calibrated EMIO frame, mm) from the camera to the motor GUI.

The GUI's Tip control panel uses it for the live tip display and closed-loop moves.
Run with the emio-venv Python while the GUI is running:  ~/emio-venv/bin/python tip_feed.py [--show]
Stop with Ctrl+C.
"""
import argparse
import json
import math
import time
import urllib.error
import urllib.request

from emioapi import EmioCamera


def post(url, body):
    req = urllib.request.Request(url, json.dumps(body).encode(), {"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=1).read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--api", default="http://localhost:8000/api")
    ap.add_argument("--show", action="store_true", help="show the camera windows")
    args = ap.parse_args()

    cam = EmioCamera(show=args.show, track_markers=True)
    if not cam.open():
        raise SystemExit("Could not open the camera (close track_live.py / Emio Labs / collect_tip_data.py)")
    print(f"Streaming tip position to {args.api}/tip. Ctrl+C to stop.")
    last, warned, sent = None, False, 0
    try:
        while cam.is_running:
            cam.update()
            pos = [p[:3] for p in cam.trackers_pos]
            if pos:
                # one marker expected; if more, follow the one closest to the last tip
                tip = min(pos, key=lambda p: math.dist(p, last)) if last is not None else pos[0]
                last = tip
                body = {"x": tip[0], "y": tip[1], "z": tip[2], "markers": len(pos)}
            else:
                body = {"markers": 0}
            try:
                post(f"{args.api}/tip", body)
                warned = False
                sent += 1
                if sent % 30 == 0:
                    print(f"\r{body.get('markers')} marker(s)  " +
                          (f"tip ({body['x']:.1f}, {body['y']:.1f}, {body['z']:.1f})" if pos else "tip not seen") +
                          " " * 10, end="", flush=True)
            except (urllib.error.URLError, OSError):
                if not warned:
                    print(f"\nMotor GUI not reachable at {args.api}; retrying...")
                    warned = True
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        cam.close()


if __name__ == "__main__":
    main()
