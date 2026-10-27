#!/usr/bin/env python3
"""Stream the tip position (green marker, calibrated EMIO frame, mm) from the camera to the motor GUI.

The GUI's Tip control panel uses it for the live tip display and closed-loop moves, and shows the camera image
(with the tracked marker outlined) as a live view. It also finds the obstacle rod by its colour (rod_detect.py,
orange unless another colour is picked in the GUI) and sends it to the GUI, which plans tip moves around it.
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
import numpy as np
from emioapi import EmioCamera

import rod_detect

ROD_DETECT_S = 0.15  # look for the rod this often
ROD_POST_S = 0.3     # and send it to the GUI this often


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


class RodLink:
    """Finds the rod in the camera images and keeps the GUI up to date; handles colour picks and marked ends
    requested from the GUI (as fractions of the image)."""

    def __init__(self, api, cam, preset):
        self.url, self.cam, self.preset = f"{api}/obstacle", cam, rod_detect.PRESETS[preset]
        self.geom = rod_detect.CameraGeometry.from_emio(cam)
        self.tracker = rod_detect.RodTracker()
        self.hsv, self.det, self.mask = None, None, None
        self.t_detect = self.t_post = 0.0
        self.reply = {}  # extra fields for the next post (pick / marking results)

    def step(self, view_frame):
        now = time.monotonic()
        if now - self.t_detect >= ROD_DETECT_S:
            self.t_detect = now
            self.det, self.mask = rod_detect.detect(self.cam.hsv_frame, self.cam.depth_frame, self.hsv or self.preset, self.geom)
            self.tracker.update(self.det)
        if view_frame is not None and self.mask is not None:
            rod_detect.draw(view_frame, self.mask, self.det, self.tracker.rod(), self.geom)
        if now - self.t_post >= ROD_POST_S:
            self.t_post = now
            body = {"rod": self.tracker.rod(), "seen": self.det is not None, **self.reply}
            self.reply = {}
            try:
                req = urllib.request.Request(self.url, json.dumps(body).encode(), {"Content-Type": "application/json"})
                cfg = json.loads(urllib.request.urlopen(req, timeout=1).read())
            except (urllib.error.URLError, OSError, ValueError):
                return  # GUI not reachable (the tip loop reports it) or an older GUI without obstacles
            if cfg.get("hsv") != self.hsv:
                self.hsv = cfg.get("hsv")
                self.tracker.reset()
            if cfg.get("reset"):
                self.tracker.reset()
            if cfg.get("pick"):
                self._pick(*cfg["pick"])
            if cfg.get("mark"):
                self._mark(cfg["mark"])

    def _pixel(self, fu, fv):
        return min(self.geom.w - 1, max(0, round(fu * self.geom.w))), min(self.geom.h - 1, max(0, round(fv * self.geom.h)))

    def _pick(self, fu, fv):
        try:
            self.reply = {"hsv_picked": rod_detect.hsv_range_at(self.cam.hsv_frame, *self._pixel(fu, fv))}
            print(f"\nRod colour picked: {self.reply['hsv_picked']}")
        except ValueError as e:
            self.reply = {"error": str(e)}

    def _mark(self, mark):
        depth, r = self.cam.depth_frame, float(mark["radius"])
        ends = []
        for fu, fv in mark["points"]:
            x, y = self._pixel(fu, fv)
            patch = depth[max(0, y - 4):y + 5, max(0, x - 4):x + 5].astype(float)
            patch = patch[(patch >= rod_detect.DEPTH_RANGE_MM[0]) & (patch <= rod_detect.DEPTH_RANGE_MM[1])]
            if not len(patch):
                self.reply = {"error": "no depth reading at one of the ends; click a little further along the rod"}
                return
            ends.append(self.geom.to_world(float(x), float(y), float(np.percentile(patch, 25))))  # rod is in front
        ends = np.array(ends)
        u = ends[1] - ends[0]
        if np.linalg.norm(u) < 10:
            self.reply = {"error": "the two ends are less than 10 mm apart"}
            return
        u /= np.linalg.norm(u)
        view = ends.mean(axis=0) - self.geom.centre
        away = view - (view @ u) * u
        ends += rod_detect.SURFACE_SHIFT * r * away / np.linalg.norm(away)  # measured on the near surface
        self.reply = {"marked": {"p0": ends[0].round(1).tolist(), "p1": ends[1].round(1).tolist(), "radius": r}}
        print(f"\nRod marked: {self.reply['marked']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--api", default="http://localhost:8000/api")
    ap.add_argument("--show", action="store_true", help="show the camera windows")
    ap.add_argument("--view-fps", type=float, default=10, help="live view frame rate in the GUI (0 = off)")
    ap.add_argument("--view-width", type=int, default=640, help="live view image width in pixels")
    ap.add_argument("--rod-color", default="orange", choices=sorted(rod_detect.PRESETS),
                    help="obstacle rod colour, until one is picked in the GUI")
    ap.add_argument("--no-rod", action="store_true", help="don't look for the obstacle rod")
    args = ap.parse_args()

    cam = EmioCamera(show=args.show, track_markers=True)
    if not cam.open():
        raise SystemExit("Could not open the camera (close track_live.py / Emio Labs / collect_tip_data.py)")
    view = None
    if args.view_fps > 0:
        view = ViewSender(f"{args.api}/camera/frame", args.view_fps, args.view_width)
        view.start()
    rod = None if args.no_rod else RodLink(args.api, cam, args.rod_color)
    print(f"Streaming tip position to {args.api}/tip" + (" and the live view" if view else "") + ". Ctrl+C to stop.")
    last, warned, sent = None, False, 0
    try:
        while cam.is_running:
            cam.update()
            pos = [p[:3] for p in cam.trackers_pos]
            frame = cam.frame
            shown = frame.copy() if view is not None and frame is not None else None  # copy: the camera reuses its buffers
            if rod is not None and frame is not None:
                rod.step(shown)
            if shown is not None:
                view.submit(shown)
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
