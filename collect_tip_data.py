#!/usr/bin/env python3
"""Collect (motor angles -> tip position) data for the cable-driven appendage on EMIO.

Motors are driven through the motor GUI's API (so its zeros, soft limits and Esc/TORQUE OFF still apply),
and the tip (one green marker) is measured with the calibrated EMIO camera.

Opposing cables: motors 0/2 form pair A, motors 1/3 form pair B. Each pair has one bend value
d in [-1, 1]: d > 0 pulls the first motor to d * its max pull (GUI soft max minus taut zero), d < 0 pulls the
opposite motor, and the motor not pulling rests released at absolute 0. So within a pair only one motor pulls
at a time (at most 2 active overall), and moves that cross d = 0 release one cable before pulling the other.
Moves are done smoothly by the motor GUI (/api/tip/move_bend), so the GUI must be up to date.
Samples lie on a grid inside the circle dA^2 + dB^2 <= radius^2.

Usage (motor GUI running at http://localhost:8000, emio-venv Python for the camera):
  ~/emio-venv/bin/python collect_tip_data.py --check              # no motion: check GUI, motors, camera
  ~/emio-venv/bin/python collect_tip_data.py --dry-run            # print the plan, no motion
  ~/emio-venv/bin/python collect_tip_data.py --radius 0.5         # small first run
  ~/emio-venv/bin/python collect_tip_data.py                      # full run (radius 1.0)
Ctrl+C at any time releases all motors to rest and stops.
"""
import argparse
import csv
import json
import math
import random
import shutil
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

# Max safe pull from each motor's taut zero (deg). Only used by --dry-run: a real run reads it from the
# motor GUI (soft max minus taut zero), the same mapping the GUI's tip control uses.
MAX_SAFE_DEG = {0: 146.69, 1: 138.25, 2: 102.4, 3: 113.4}
PAIRS = {"A": (0, 2), "B": (1, 3)}   # (pulled when d > 0, pulled when d < 0)
REST_ABS_RAD = 0.0                    # where a released motor rests (absolute angle)
MA_PER_UNIT = 2.69                    # XM430 current unit
MAX_ERROR_DEG = 8.0                   # settled further than this from the goal = blocked
CALIB_FILE = Path.home() / ".config" / "emioapi" / "camera_2d_points.csv"
GUI_SETTINGS = Path(__file__).resolve().parent / "motor_gui" / "data" / "settings.json"


class Abort(Exception):
    pass


class ForceLimit(Abort):
    """A motor passed the soft current threshold: back off and skip the sample instead of aborting."""
    def __init__(self, motor, ma):
        super().__init__(f"Motor {motor} current {ma:.0f} mA")
        self.motor, self.ma = motor, ma


# ---------------------------------------------------------------- motor GUI API
class Gui:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def _req(self, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(f"{self.url}/{path}", data, {"Content-Type": "application/json"})
        try:
            return json.load(urllib.request.urlopen(req, timeout=5))
        except urllib.error.HTTPError as e:
            raise Abort(f"GUI API {path}: {e.code} {e.read().decode(errors='replace')}") from None
        except urllib.error.URLError as e:
            raise Abort(f"Motor GUI not reachable at {self.url} ({e.reason}). Is it running?") from None

    def state(self):
        return self._req("state")

    def torque(self, i, on):
        return self._req(f"motor/{i}/torque", {"on": on})

    def goal_abs(self, i, rad):
        return self._req(f"motor/{i}/goal", {"angle_rad": rad})

    def goal_rel(self, i, rad):
        return self._req(f"motor/{i}/goal", {"angle_rad": rad, "relative": True})


def pair_targets(d):
    """{pair: d} -> {motor: ("rel", rad) | ("abs", rad)}"""
    out = {}
    for name, (pos, neg) in PAIRS.items():
        v = d[name]
        for m, active in ((pos, v > 0), (neg, v < 0)):
            out[m] = ("rel", math.radians(abs(v) * MAX_SAFE_DEG[m])) if active else ("abs", REST_ABS_RAD)
    return out


def check_motors(s, max_ma, soft_ma=None):
    for m in s["motors"]:
        if not m["online"]:
            raise Abort(f"Motor {m['id']} not responding")
        if m.get("hw_error"):
            raise Abort(f"Motor {m['id']} hardware error {m['hw_error']}")
        if m.get("overridden"):
            raise Abort(f"Motor {m['id']} goal overridden by another program (Emio Labs?)")
        ma = abs(m["current_raw"]) * MA_PER_UNIT
        if ma > max_ma:
            raise Abort(f"Motor {m['id']} current {ma:.0f} mA > {max_ma} mA")
        if soft_ma is not None and ma > soft_ma:
            raise ForceLimit(m["id"], ma)


def wait_settled(gui, max_ma, soft_ma=None, timeout=8.0, stable_pulses=2, window=6):
    """Wait until every motor has stopped moving: its position changed by <= stable_pulses over the last
    `window` polls (50 ms apart). Loaded motors settle a little short of their goal (proportional-only
    position control), so reaching the goal exactly is not required; a motor that stops more than
    MAX_ERROR_DEG away from its goal counts as blocked."""
    t0, hist = time.monotonic(), []
    while True:
        s = gui.state()
        check_motors(s, max_ma, soft_ma)
        hist = (hist + [[m["pulse"] for m in s["motors"]]])[-window:]
        stable = len(hist) == window and all(max(c) - min(c) <= stable_pulses for c in zip(*hist))
        if stable:
            far = [m["id"] for m in s["motors"] if abs(m["goal_pulse"] - m["pulse"]) > MAX_ERROR_DEG / 360 * 4096]
            if far:
                raise Abort(f"Motors {far} stopped more than {MAX_ERROR_DEG}° from their goal (blocked?)")
            return s
        if time.monotonic() - t0 > timeout:
            moving = [m["id"] for m, c in zip(s["motors"], zip(*hist)) if max(c) - min(c) > stable_pulses]
            raise Abort(f"Motors {moving} still moving after {timeout} s")
        time.sleep(0.05)


def move(gui, cur, tgt, max_ma, soft_ma=None):
    """Smooth move to bend values tgt, done by the motor GUI (/api/tip/move_bend): goals are streamed with
    an ease-in/out, a pair that changes sign releases one cable before the opposite pulls, and above soft_ma
    it backs off and stops short. Returns (pose reached, ForceLimit or None)."""
    r = gui._req("tip/move_bend", {"A": tgt["A"], "B": tgt["B"], "soft_ma": soft_ma, "hard_ma": max_ma})
    lim = r["limited"]
    return r["pose"], (ForceLimit(lim["motor"], lim["ma"]) if lim else None)


def release_all(gui):
    for m in range(4):
        try:
            gui.goal_abs(m, REST_ABS_RAD)
        except Abort:
            pass


# ---------------------------------------------------------------- camera
def open_camera(show):
    from emioapi import EmioCamera
    cam = EmioCamera(show=show, track_markers=True)
    if not cam.open():
        raise Abort("Could not open the camera (close track_live.py / Emio Labs)")
    for _ in range(15):  # let exposure settle
        cam.update()
    return cam


def measure_tip(cam, frames, prev=None):
    """Average the tip over `frames` frames. Returns (xyz or None, std_mm, frames_detected, max_markers_seen)."""
    for _ in range(3):  # drop frames buffered while moving
        cam.update()
    pts, max_seen = [], 0
    for _ in range(frames):
        cam.update()
        pos = [p[:3] for p in cam.trackers_pos]
        max_seen = max(max_seen, len(pos))
        if pos:
            ref = prev if prev is not None else (pts[-1] if pts else None)
            pts.append(min(pos, key=lambda p: math.dist(p, ref)) if ref is not None and len(pos) > 1 else pos[0])
    if not pts:
        return None, float("nan"), 0, max_seen
    mean = [sum(p[a] for p in pts) / len(pts) for a in range(3)]
    std = math.sqrt(sum(math.dist(p, mean) ** 2 for p in pts) / len(pts))
    return mean, std, len(pts), max_seen


# ---------------------------------------------------------------- plan
def grid_plan(n, radius):
    vals = [radius * (-1 + 2 * k / (n - 1)) for k in range(n)]
    plan = []
    for r, a in enumerate(vals):  # serpentine: neighbouring samples are one grid step apart
        row = [{"A": a, "B": b} for b in (vals if r % 2 == 0 else vals[::-1])]
        plan += [d for d in row if math.hypot(d["A"], d["B"]) <= radius + 1e-9]
    return plan


def main():
    global MAX_ERROR_DEG
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--api", default="http://localhost:8000/api")
    ap.add_argument("--grid", type=int, default=9, help="grid points per axis (default 9)")
    ap.add_argument("--radius", type=float, default=1.0, help="max bend as a fraction of max safe angle (0-1)")
    ap.add_argument("--revisits", type=int, default=20, help="random revisits of grid points, for hysteresis")
    ap.add_argument("--frames", type=int, default=10, help="camera frames averaged per sample")
    ap.add_argument("--settle", type=float, default=0.8, help="extra wait after motors stop (s)")
    ap.add_argument("--max-current-ma", type=float, default=1100, help="abort above this motor current (default 1100)")
    ap.add_argument("--soft-current-ma", type=float, default=1000,
                    help="above this, back off and log the sample as force-limited instead of aborting (default 1000)")
    ap.add_argument("--max-error-deg", type=float, default=MAX_ERROR_DEG,
                    help="abort if a motor settles further than this from its goal (default 8)")
    ap.add_argument("--out", default=None, help="output CSV (default data/tip_<time>.csv)")
    ap.add_argument("--show", action="store_true", help="show camera windows")
    ap.add_argument("--dry-run", action="store_true", help="print the plan only")
    ap.add_argument("--check", action="store_true", help="check GUI, motors and camera without moving")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    if not 0 < args.radius <= 1:
        sys.exit("--radius must be in (0, 1]")
    if args.soft_current_ma >= args.max_current_ma:
        sys.exit("--soft-current-ma must be below --max-current-ma")
    MAX_ERROR_DEG = args.max_error_deg

    plan = [("grid", d) for d in grid_plan(args.grid, args.radius)]
    rng = random.Random(args.seed)
    plan += [("revisit", d) for _, d in rng.sample(plan, min(args.revisits, len(plan)))]

    if args.dry_run:
        for k, (phase, d) in enumerate(plan):
            t = pair_targets(d)
            desc = "  ".join(f"m{m}: {'%+6.1f° rel' % math.degrees(r) if kd == 'rel' else '  rest    '}"
                             for m, (kd, r) in sorted(t.items()))
            print(f"{k:3d} {phase:7s} dA={d['A']:+.2f} dB={d['B']:+.2f}  {desc}")
        print(f"{len(plan)} samples (~{len(plan) * 3 / 60:.0f} min)")
        return

    gui = Gui(args.api)
    cam = None
    moving = False  # only release the motors on errors once this script has started driving them
    try:
        s = gui.state()
        if not s["connected"]:
            raise Abort("Motor GUI is not connected to the EMIO")
        uncal = [m["id"] for m in s["motors"] if m["settings"]["zero_rad"] is None]
        if uncal:
            raise Abort(f"Motors {uncal} have no taut zero; calibrate them in the GUI first")
        check_motors(s, args.max_current_ma)
        for m in s["motors"]:
            MAX_SAFE_DEG[m["id"]] = m["settings"]["max_deg"] - math.degrees(m["settings"]["zero_rad"])
        print("Max pull from taut zero (from GUI soft limits): "
              + ", ".join(f"m{m} {v:.1f}°" for m, v in MAX_SAFE_DEG.items()))
        cam = open_camera(args.show)
        tip, std, n, seen = measure_tip(cam, args.frames)
        print(f"Camera: {seen} marker(s) seen; tip {['%.1f' % v for v in tip] if tip else None} (std {std:.2f} mm)")
        if seen != 1:
            print("WARNING: expected exactly one green marker (the tip) in view")
        if args.check:
            print("Check OK" if seen == 1 else "Check finished with warnings")
            return

        out = Path(args.out or Path(__file__).resolve().parent / "data" / f"tip_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        out.parent.mkdir(parents=True, exist_ok=True)
        for src, suffix in ((CALIB_FILE, "_camera_calibration.csv"), (GUI_SETTINGS, "_motor_settings.json")):
            if src.exists():
                shutil.copy(src, out.with_name(out.stem + suffix))

        moving = True
        for m in range(4):
            gui.torque(m, True)  # holds the current position
        cur = {"A": 0.0, "B": 0.0}
        release_all(gui)
        wait_settled(gui, args.max_current_ma)

        # dA/dB = pose actually measured; target_* = planned sample (differs when force-limited)
        cols = ["t", "idx", "phase", "dA", "dB", "target_dA", "target_dB", "force_limited_motor", "limit_current_ma"]
        for m in range(4):
            cols += [f"m{m}_cmd_rel_deg", f"m{m}_rel_deg", f"m{m}_abs_deg", f"m{m}_pulse", f"m{m}_current_ma"]
        cols += ["tip_x", "tip_y", "tip_z", "tip_std_mm", "tip_frames", "markers_seen"]
        print(f"Collecting {len(plan)} samples -> {out}")
        prev_tip = tip
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            n_limited = 0
            for k, (phase, d) in enumerate(plan):
                cur, lim = move(gui, cur, d, args.max_current_ma, args.soft_current_ma)
                n_limited += lim is not None
                time.sleep(args.settle)
                tip, std, n, seen = measure_tip(cam, args.frames, prev_tip)
                s = wait_settled(gui, args.max_current_ma)
                tgt = pair_targets(cur)
                row = [f"{time.time():.3f}", k, phase, f"{cur['A']:.4f}", f"{cur['B']:.4f}", f"{d['A']:.4f}", f"{d['B']:.4f}",
                       lim.motor if lim else "", f"{lim.ma:.0f}" if lim else ""]
                for m in s["motors"]:
                    kind, rad = tgt[m["id"]]
                    cmd = math.degrees(rad) if kind == "rel" else math.degrees(REST_ABS_RAD - m["settings"]["zero_rad"])
                    row += [f"{cmd:.3f}", f"{math.degrees(m['angle_rel_rad']):.3f}", f"{math.degrees(m['angle_rad']):.3f}",
                            m["pulse"], f"{m['current_raw'] * MA_PER_UNIT:.0f}"]
                row += [f"{v:.2f}" for v in tip] if tip else ["", "", ""]
                row += [f"{std:.2f}", n, seen]
                w.writerow(row)
                f.flush()
                if tip:
                    prev_tip = tip
                limited = (f"FORCE LIMITED (motor {lim.motor}, {lim.ma:.0f} mA), backed off to "
                           f"dA={cur['A']:+.2f} dB={cur['B']:+.2f}; ") if lim else ""
                print(f"[{k + 1}/{len(plan)}] {phase} dA={d['A']:+.2f} dB={d['B']:+.2f} -> {limited}"
                      + (f"tip ({tip[0]:.1f}, {tip[1]:.1f}, {tip[2]:.1f}) ±{std:.1f} mm" if tip else "tip NOT SEEN"))
            move(gui, cur, {"A": 0.0, "B": 0.0}, args.max_current_ma)
        print(f"Done: {out}  ({n_limited} force-limited samples)")
    except KeyboardInterrupt:
        print("\nInterrupted" + (": releasing motors to rest" if moving else ""))
        if moving:
            release_all(gui)
    except Abort as e:
        print(f"\nABORT: {e}" + ("\nReleasing motors to rest." if moving else ""))
        if moving:
            release_all(gui)
        sys.exit(1)
    finally:
        if cam:
            cam.close()


if __name__ == "__main__":
    main()
