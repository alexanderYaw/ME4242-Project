#!/usr/bin/env python3
"""Stream the tip position (green marker, calibrated EMIO frame, mm) from the camera to the motor GUI.

The GUI's Tip control panel uses it for the live tip display and closed-loop moves, and shows the camera image
(with the tracked marker outlined) as a live view. It also finds the obstacle rods by their colour (rod_detect.py,
orange and red unless another colour is picked in the GUI), and other objects of those colours (a tray, a button,
...), and sends them to the GUI, which shows them (they are not avoided), and sends a coarse 3D point cloud of the workspace for the GUI's 3D view.
Run with the emio-venv Python while the GUI is running:  ~/emio-venv/bin/python tip_feed.py [--show]
Stop with Ctrl+C.
"""
import os

# opencv-python bundles an old OpenBLAS (0.3.3) that starts a busy-waiting thread per core: ~9 cores at 100%, which
# starves the motor GUI (its serial replies then time out: "There is no status packet!"). Set before cv2 / numpy load.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse  # noqa: E402
import json
import math
import signal
import struct
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np
from emioapi import EmioCamera

import rod_detect

ROD_DETECT_S = 0.15  # look for the rods this often
ROD_POST_S = 0.3     # and send them to the GUI this often
CLOUD_SCALE = 4      # point cloud coordinates are sent as int16 in 1/4 mm
CV_THREADS = 2       # OpenCV's own pool (default: a thread per core) also spins between emioapi's small per-frame ops


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


class CloudSender(threading.Thread):
    """Posts a coarse point cloud of the workspace (every `step`-th pixel with depth, calibrated EMIO frame) to the
    GUI's 3D view in the background. Points of the obstacle's colour are flagged.

    Body: uint32 header length, JSON header (n, scale, camera pose), padded to 4 bytes; then int16 x, y, z
    (n x 3, in 1/scale mm), uint8 r, g, b (n x 3) and uint8 flags (n; 1 = obstacle colour)."""

    def __init__(self, url, fps, geom, step):
        super().__init__(daemon=True)
        self.url, self.period, self.geom, self.step = url, 1 / fps, geom, step
        self.data, self.ready = None, threading.Event()
        h, w = geom.h, geom.w
        v, u = np.mgrid[0:h:step, 0:w:step]
        self.u, self.v = u.astype(float), v.astype(float)
        cam = {"centre": geom.t.round(2).tolist(), "R": geom.R.round(5).tolist(),
               "fx": geom.fx, "fy": geom.fy, "ppx": geom.ppx, "ppy": geom.ppy, "w": w, "h": h}
        self.cam = cam

    def submit(self, hsv, depth, mask):
        """Subsampled copies of the latest images (the camera reuses its buffers)."""
        if self.ready.is_set():
            return  # the last one isn't sent yet
        s = self.step
        self.data = (hsv[::s, ::s].copy(), depth[::s, ::s].copy(), None if mask is None else mask[::s, ::s].copy())
        self.ready.set()

    def run(self):
        while True:
            self.ready.wait()
            t0 = time.monotonic()
            hsv, depth, mask = self.data
            self.ready.clear()
            z = depth.astype(float)
            ok = (z >= rod_detect.DEPTH_RANGE_MM[0]) & (z <= rod_detect.DEPTH_RANGE_MM[1])
            W = self.geom.to_world(self.u[ok], self.v[ok], z[ok])
            rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[ok]
            flags = (mask[ok] > 0).astype(np.uint8) if mask is not None else np.zeros(len(W), np.uint8)
            head = json.dumps({"n": len(W), "scale": CLOUD_SCALE, "camera": self.cam}).encode()
            head += b" " * (-(len(head) + 4) % 4)
            xyz = np.clip(np.round(W * CLOUD_SCALE), -32767, 32767).astype("<i2")
            body = struct.pack("<I", len(head)) + head + xyz.tobytes() + rgb.astype(np.uint8).tobytes() + flags.tobytes()
            req = urllib.request.Request(self.url, body, {"Content-Type": "application/octet-stream"})
            try:
                urllib.request.urlopen(req, timeout=2).read()
            except (urllib.error.URLError, OSError):
                time.sleep(1.0)  # GUI not reachable (or older, without the 3D view)
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))


class RodLink(threading.Thread):
    """Finds the rods and other objects in the camera images in the background (so tracking isn't slowed down) and
    keeps the GUI up to date; handles colour picks and marked ends requested from the GUI (as fractions of the image).
    `kill -USR1 <pid>` saves the next raw image pair (HSV + depth, no overlay) to snapshots/, for tuning rod_detect."""

    OBJECTS_RESEND_S = 5.0  # the objects' meshes are sent when they change, and this often anyway

    def __init__(self, api, cam, colours):
        super().__init__(daemon=True)
        self.url, self.cam = f"{api}/obstacle", cam
        self.presets = {c: rod_detect.PRESETS[c] for c in colours}
        self.geom = rod_detect.CameraGeometry.from_emio(cam)
        self.tracker, self.obj_tracker = rod_detect.RodTracker(), rod_detect.ObjectTracker()
        self.hsv = None  # colour picked in the GUI: replaces the presets
        # latest results, for the live view / cloud
        self.dets, self.masks, self.rods, self.objects, self.mask = [], {}, [], [], None
        self.frames, self.ready = None, threading.Event()
        self.t_submit = self.t_post = self.t_objects = 0.0
        self.sent_version = None  # objects version last sent
        self.reply = {}  # extra fields for the next post (pick / marking results)
        self.snapshot = False

    def step(self, view_frame):
        """Main loop: hand the latest images to the detector now and then, and draw its latest results."""
        now = time.monotonic()
        if not self.ready.is_set() and now - self.t_submit >= ROD_DETECT_S:
            self.t_submit = now
            self.frames = (self.cam.hsv_frame.copy(), self.cam.depth_frame.copy())  # the camera reuses its buffers
            self.ready.set()
        if view_frame is not None:
            rod_detect.draw(view_frame, self.masks, self.dets, self.rods, self.geom, self.objects)

    def run(self):
        while True:
            self.ready.wait()
            hsv, depth = self.frames
            if self.snapshot:
                self.snapshot = False
                path = Path(__file__).resolve().parent / "snapshots" / time.strftime("frame_%Y%m%d_%H%M%S.npz")
                path.parent.mkdir(exist_ok=True)
                np.savez_compressed(path, hsv=hsv, depth=depth)
                print(f"\nSaved {path}")
            dets, objects, masks = rod_detect.detect(hsv, depth, {"picked": self.hsv} if self.hsv else self.presets,
                                                     self.geom)
            self.tracker.update(dets)
            self.obj_tracker.update(objects)
            mask = None
            for m in masks.values():
                mask = m if mask is None else mask | m
            self.dets, self.masks, self.rods, self.mask = dets, masks, self.tracker.rods(), mask
            self.objects = self.obj_tracker.objects()
            if time.monotonic() - self.t_post >= ROD_POST_S:
                self.t_post = time.monotonic()
                self._post(hsv, depth)
            self.ready.clear()

    def _post(self, hsv, depth):
        body = {"rods": self.rods, "seen": bool(self.dets), **self.reply}
        version = self.obj_tracker.version
        if version != self.sent_version or time.monotonic() - self.t_objects >= self.OBJECTS_RESEND_S:
            body["objects"] = [{**{k: v for k, v in o.items() if k != "img_outline"},
                                "pockets": [{k: v for k, v in p.items() if k != "img_outline"} for p in o["pockets"]]}
                               for o in self.objects]
        self.reply = {}
        try:
            req = urllib.request.Request(self.url, json.dumps(body).encode(), {"Content-Type": "application/json"})
            cfg = json.loads(urllib.request.urlopen(req, timeout=1).read())
        except (urllib.error.URLError, OSError, ValueError):
            return  # GUI not reachable (the tip loop reports it) or an older GUI without obstacles
        if "objects" in body:
            self.sent_version, self.t_objects = version, time.monotonic()
        if cfg.get("hsv") != self.hsv or cfg.get("reset"):
            self.hsv = cfg.get("hsv")
            self.tracker.reset()
            self.obj_tracker.reset()
        if cfg.get("pick"):
            self._pick(hsv, *cfg["pick"])
        if cfg.get("mark"):
            self._mark(depth, cfg["mark"])

    def _pixel(self, fu, fv):
        return min(self.geom.w - 1, max(0, round(fu * self.geom.w))), min(self.geom.h - 1, max(0, round(fv * self.geom.h)))

    def _pick(self, hsv, fu, fv):
        try:
            self.reply = {"hsv_picked": rod_detect.hsv_range_at(hsv, *self._pixel(fu, fv))}
            print(f"\nRod colour picked: {self.reply['hsv_picked']}")
        except ValueError as e:
            self.reply = {"error": str(e)}

    def _mark(self, depth, mark):
        r = float(mark["radius"])
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
    ap.add_argument("--rod-color", nargs="+", default=list(rod_detect.DEFAULT_COLOURS), choices=sorted(rod_detect.PRESETS),
                    help="obstacle rod colours, until one is picked in the GUI")
    ap.add_argument("--no-rod", action="store_true", help="don't look for obstacle rods")
    ap.add_argument("--cloud-fps", type=float, default=2, help="3D view point cloud rate (0 = off)")
    ap.add_argument("--cloud-step", type=int, default=4, help="point cloud: use every n-th pixel")
    args = ap.parse_args()

    cv2.setNumThreads(CV_THREADS)
    cam = EmioCamera(show=args.show, track_markers=True)
    if not cam.open():
        raise SystemExit("Could not open the camera (close track_live.py / Emio Labs / collect_tip_data.py)")
    view = None
    if args.view_fps > 0:
        view = ViewSender(f"{args.api}/camera/frame", args.view_fps, args.view_width)
        view.start()
    rod = None if args.no_rod else RodLink(args.api, cam, args.rod_color)
    if rod is not None:
        rod.start()
        signal.signal(signal.SIGUSR1, lambda *_: setattr(rod, "snapshot", True))
    cloud = None
    if args.cloud_fps > 0:
        cloud = CloudSender(f"{args.api}/camera/cloud", args.cloud_fps,
                            rod.geom if rod else rod_detect.CameraGeometry.from_emio(cam), max(1, args.cloud_step))
        cloud.start()
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
            if cloud is not None and frame is not None:
                cloud.submit(cam.hsv_frame, cam.depth_frame, rod.mask if rod else None)
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
