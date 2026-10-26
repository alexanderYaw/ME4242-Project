#!/usr/bin/env python3
"""Stream the tip position (green marker, calibrated EMIO frame, mm) from the camera to the motor GUI.

The GUI's Tip control panel uses it for the live tip display and closed-loop moves, and shows the camera image
(with the tracked marker outlined) as a live view.
Run with the emio-venv Python while the GUI is running:  ~/emio-venv/bin/python tip_feed.py [--show]
Stop with Ctrl+C.
"""
import argparse
import json
import math
import threading
import time
import urllib.error
import urllib.request

import cv2
from emioapi import EmioCamera


def post(url, body):
    req = urllib.request.Request(url, json.dumps(body).encode(), {"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=1).read()


class ViewSender(threading.Thread):
    """Posts the latest camera image to the GUI's live view in the background, so tracking isn't slowed down."""

    def __init__(self, url, fps, width):
        super().__init__(daemon=True)
        self.url, self.period, self.width = url, 1 / fps, width
        self.frame, self.ready = None, threading.Event()

    def submit(self, frame):
        self.frame = frame
        self.ready.set()

    def run(self):
        while True:
            self.ready.wait()
            self.ready.clear()
            t0, frame = time.monotonic(), self.frame
            h, w = frame.shape[:2]
            if w > self.width:
                frame = cv2.resize(frame, (self.width, round(h * self.width / w)), interpolation=cv2.INTER_AREA)
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                req = urllib.request.Request(self.url, jpg.tobytes(), {"Content-Type": "image/jpeg"})
                try:
                    urllib.request.urlopen(req, timeout=1).read()
                except (urllib.error.URLError, OSError):
                    time.sleep(0.5)  # GUI not reachable; the tip loop reports it
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--api", default="http://localhost:8000/api")
    ap.add_argument("--show", action="store_true", help="show the camera windows")
    ap.add_argument("--view-fps", type=float, default=10, help="live view frame rate in the GUI (0 = off)")
    ap.add_argument("--view-width", type=int, default=640, help="live view image width in pixels")
    args = ap.parse_args()

    cam = EmioCamera(show=args.show, track_markers=True)
    if not cam.open():
        raise SystemExit("Could not open the camera (close track_live.py / Emio Labs / collect_tip_data.py)")
    view = None
    if args.view_fps > 0:
        view = ViewSender(f"{args.api}/camera/frame", args.view_fps, args.view_width)
        view.start()
    print(f"Streaming tip position to {args.api}/tip" + (" and the live view" if view else "") + ". Ctrl+C to stop.")
    last, warned, sent = None, False, 0
    try:
        while cam.is_running:
            cam.update()
            pos = [p[:3] for p in cam.trackers_pos]
            frame = cam.frame
            if view is not None and frame is not None:
                view.submit(frame.copy())  # copy: the camera reuses its buffers
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
