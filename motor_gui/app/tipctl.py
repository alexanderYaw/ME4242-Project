"""Tip control: tip model (bend values -> tip position), live tip feed from the camera, and moves to a
target tip position (x, z) with optional closed-loop correction from the camera.

Bend values follow collect_tip_data.py: motors 0/2 form pair A and motors 1/3 pair B. Each pair has one
value d in [-1, 1]; d > 0 pulls the first motor to d * its max pull (soft max minus taut zero), d < 0 pulls
the opposite motor, and the other motor rests released at absolute 0. Moves that cross d = 0 release one
cable before pulling the other, and |(dA, dB)| <= 1.
"""
import collections
import json
import math
import threading
import time
from pathlib import Path

import numpy as np

import motors

PAIRS = {"A": (0, 2), "B": (1, 3)}
QUADRANTS = {"++": (1, 1), "+-": (1, -1), "-+": (-1, 1), "--": (-1, -1)}
REST_ABS_RAD = 0.0
MAX_SUBSTEP = 0.1        # largest change in d per intermediate move
SOFT_MA = 1000           # above this: back off and stop short
HARD_MA = 1100           # above this: release all motors and abort
MA_PER_UNIT = 2.69
MAX_ERROR_DEG = 8.0      # motor settled further than this from its goal = blocked
FEED_MAX_AGE_S = 0.5
TOLERANCE_MM = 3.0
MAX_ITERATIONS = 8
MAX_CORRECTION = 0.15    # largest change in d per closed-loop correction
OUTSIDE_MM = 15.0        # target further than this from the model's reach = outside the workspace


class Stopped(Exception):
    pass


class Fail(Exception):
    pass


class ForceLimit(Exception):
    def __init__(self, motor, ma):
        super().__init__(f"motor {motor} at {ma:.0f} mA")
        self.motor, self.ma = motor, ma


class TipModel:
    def __init__(self, path):
        m = json.loads(Path(path).read_text())
        self.info = m
        self.deg = m["degree"]
        self.coeffs = {q: np.array(c) for q, c in m["coeffs"].items()}
        g = np.linspace(-1, 1, 161)
        A, B = np.meshgrid(g, g)
        D = np.column_stack([A.ravel(), B.ravel()])
        self.grid_d = D[np.hypot(D[:, 0], D[:, 1]) <= 1 + 1e-9]
        self.grid_xz = self.predict(self.grid_d)[:, [0, 2]]
        t = np.linspace(0, 2 * math.pi, 181)
        self.outline_xz = self.predict(np.column_stack([np.cos(t), np.sin(t)]))[:, [0, 2]]

    def _features(self, D):
        a, b = D[:, 0], D[:, 1]
        return np.column_stack([a ** i * b ** j for i in range(self.deg + 1) for j in range(self.deg + 1 - i)])

    def predict(self, D):
        D = np.atleast_2d(np.asarray(D, dtype=float))
        out = np.zeros((len(D), 3))
        for q, (sa, sb) in QUADRANTS.items():
            m = (D[:, 0] * sa >= -1e-12) & (D[:, 1] * sb >= -1e-12)
            if m.any():
                out[m] = self._features(D[m]) @ self.coeffs[q]
        return out

    def jacobian_xz(self, d, h=0.02):
        J = np.zeros((2, 2))
        for k in range(2):
            e = np.zeros(2)
            e[k] = h
            J[:, k] = (self.predict(d + e)[0, [0, 2]] - self.predict(d - e)[0, [0, 2]]) / (2 * h)
        return J

    def inverse(self, x, z):
        """Bend values whose predicted tip is closest to (x, z). Returns (d, predicted xz, distance mm)."""
        t = np.array([x, z])
        d = self.grid_d[np.argmin(np.linalg.norm(self.grid_xz - t, axis=1))].copy()
        for _ in range(10):  # refine with damped Gauss-Newton, staying inside the unit disk
            err = t - self.predict(d)[0, [0, 2]]
            J = self.jacobian_xz(d)
            step = np.linalg.solve(J.T @ J + 1e-3 * np.eye(2), J.T @ err)
            d = clip_disk(d + np.clip(step, -0.05, 0.05))
        p = self.predict(d)[0, [0, 2]]
        return d, p, float(np.linalg.norm(p - t))


def clip_disk(d):
    r = math.hypot(*d)
    return d / r if r > 1 else d


def waypoints(cur, tgt):
    """Path from cur to tgt: pass through d = 0 for any pair that changes sign, in steps <= MAX_SUBSTEP."""
    stops, here = [], dict(cur)
    if any(here[p] * tgt[p] < 0 for p in PAIRS):
        here = {p: (0.0 if here[p] * tgt[p] < 0 else here[p]) for p in PAIRS}
        stops.append(here)
    stops.append(dict(tgt))
    path, prev = [], dict(cur)
    for stop in stops:
        n = max(1, math.ceil(max(abs(stop[p] - prev[p]) for p in PAIRS) / MAX_SUBSTEP))
        path += [{p: prev[p] + (stop[p] - prev[p]) * k / n for p in PAIRS} for k in range(1, n + 1)]
        prev = stop
    return path


class TipController:
    def __init__(self, ctl, model_path):
        self.ctl = ctl
        self.model_path = Path(model_path)
        self._model, self._model_mtime = None, None
        self.feed = collections.deque(maxlen=200)  # (monotonic time, {x, y, z, markers})
        self.status = {"state": "idle", "target": None, "iteration": 0, "error_mm": None, "message": ""}
        self._thread = None
        self._stop = threading.Event()

    # -- model / feed -------------------------------------------------------------
    @property
    def model(self):
        try:
            mtime = self.model_path.stat().st_mtime
        except OSError:
            self._model = None
            return None
        if mtime != self._model_mtime:  # reload after fit_tip_model.py writes a new file
            self._model, self._model_mtime = TipModel(self.model_path), mtime
        return self._model

    def add_feed(self, sample):
        self.feed.append((time.monotonic(), sample))

    def latest_feed(self):
        if not self.feed:
            return None
        t, s = self.feed[-1]
        return {**s, "age_s": time.monotonic() - t}

    def measured_tip(self, window_s=0.4):
        """Average tip over the last window (after waiting for it). None if the feed is stale or the tip unseen."""
        time.sleep(window_s)
        now = time.monotonic()
        pts = [s for t, s in list(self.feed) if now - t <= window_s and s.get("markers", 0) >= 1]
        if not pts:
            return None
        return np.array([np.mean([p[k] for p in pts]) for k in ("x", "y", "z")])

    # -- pose ---------------------------------------------------------------------
    def max_pull_rad(self, i):
        s = self.ctl.settings["motors"][i]
        return math.radians(s["max_deg"]) - s["zero_rad"]

    def pose(self):
        """Current bend values from the measured motor angles."""
        d = {}
        for p, (pos, neg) in PAIRS.items():
            rel = {m: motors.to_angle(self.ctl.state[m]["pulse"]) - self.ctl.settings["motors"][m]["zero_rad"]
                   for m in (pos, neg)}
            if rel[pos] > math.radians(0.5):
                d[p] = min(1.0, rel[pos] / self.max_pull_rad(pos))
            elif rel[neg] > math.radians(0.5):
                d[p] = -min(1.0, rel[neg] / self.max_pull_rad(neg))
            else:
                d[p] = 0.0
        return d

    def targets_abs(self, d):
        out = {}
        for p, (pos, neg) in PAIRS.items():
            v = d[p]
            for m, active in ((pos, v > 0), (neg, v < 0)):
                zero = self.ctl.settings["motors"][m]["zero_rad"]
                out[m] = zero + abs(v) * self.max_pull_rad(m) if active else REST_ABS_RAD
        return out

    # -- motion -------------------------------------------------------------------
    def _goal(self, i, rad):
        lo, hi = self.ctl.limits_rad(i)
        pulse = motors.to_pulse(min(max(rad, lo), hi))
        self.ctl.bus.set_goal(i, pulse)
        self.ctl.commanded[i] = (pulse, time.monotonic())

    def _command(self, d):
        for i, rad in sorted(self.targets_abs(d).items()):
            self._goal(i, rad)

    def _wait_settled(self, soft=True, timeout=8.0, window=8):
        t0, hist = time.monotonic(), []
        while True:
            if self._stop.is_set():
                raise Stopped()
            st = self.ctl.state
            for i in motors.IDS:
                m = st.get(i)
                if m is None:
                    raise Fail(f"motor {i} not responding")
                if m["hw_error"]:
                    raise Fail(f"motor {i} hardware error {m['hw_error']}")
                if not m["torque"]:
                    raise Fail(f"motor {i} torque is off")
                ma = abs(m["current_raw"]) * MA_PER_UNIT
                if ma > HARD_MA:
                    raise Fail(f"motor {i} current {ma:.0f} mA > {HARD_MA} mA")
                if soft and ma > SOFT_MA:
                    raise ForceLimit(i, ma)
            hist = (hist + [[st[i]["pulse"] for i in motors.IDS]])[-window:]
            if len(hist) == window and all(max(c) - min(c) <= 2 for c in zip(*hist)):
                far = [i for i in motors.IDS
                       if abs(st[i]["goal_pulse"] - st[i]["pulse"]) > MAX_ERROR_DEG / 360 * 4096]
                if far:
                    raise Fail(f"motors {far} stopped more than {MAX_ERROR_DEG}° from their goal (blocked?)")
                return
            if time.monotonic() - t0 > timeout:
                raise Fail(f"motors still moving after {timeout} s")
            time.sleep(0.06)

    def _move(self, cur, tgt):
        """Move along safe waypoints. Returns (pose reached, ForceLimit or None)."""
        last = dict(cur)
        for wp in waypoints(cur, tgt):
            self._command(wp)
            try:
                self._wait_settled()
            except ForceLimit as lim:
                self._command(last)
                self._wait_settled(soft=False)
                return last, lim
            last = wp
        return dict(tgt), None

    def _release_all(self):
        for i in motors.IDS:
            try:
                self._goal(i, REST_ABS_RAD)
            except Exception:
                pass

    # -- public -------------------------------------------------------------------
    @property
    def busy(self):
        return self._thread is not None and self._thread.is_alive()

    def check_ready(self):
        if not self.ctl.bus:
            raise ValueError("Not connected")
        if self.model is None:
            raise ValueError(f"No tip model at {self.model_path}; run fit_tip_model.py first")
        if any(m["zero_rad"] is None for m in self.ctl.settings["motors"]):
            raise ValueError("Calibrate every motor's taut zero first")
        if any(self.ctl.state.get(i) is None for i in motors.IDS):
            raise ValueError("Not all motors are responding")
        if any(self.ctl.state[i]["hw_error"] for i in motors.IDS):
            raise ValueError("A motor has a hardware error; reboot it first")
        if self.busy:
            raise ValueError("Already moving; press Stop first")

    def goto(self, x, z, closed_loop=True):
        self.check_ready()
        self._stop.clear()
        self.status = {"state": "moving", "target": [x, z], "iteration": 0, "error_mm": None, "message": "starting"}
        self._thread = threading.Thread(target=self._run, args=(x, z, closed_loop), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self, x, z, closed_loop):
        st = self.status
        try:
            bus = self.ctl.bus
            bus.set_profile_velocity(self.ctl.settings["profile_velocity"])
            for i in motors.IDS:
                if not self.ctl.state[i]["torque"]:
                    bus.set_torque(i, True)  # holds the current position
                    self.ctl.commanded.pop(i, None)
            model = self.model
            target = np.array([x, z])
            d0, pred, dist = model.inverse(x, z)
            note = ""
            if dist > OUTSIDE_MM:
                note = f"target is {dist:.0f} mm outside the reachable area; going to the nearest reachable point. "
                target = pred
                st["target"] = pred.tolist()
            cur, lim = self._move(self.pose(), {"A": float(d0[0]), "B": float(d0[1])})
            if lim:
                st.update(state="limited", message=note + f"force limit ({lim}); stopped short")
                return
            fresh = self.latest_feed()
            if not closed_loop or fresh is None or fresh["age_s"] > FEED_MAX_AGE_S:
                tip = None if fresh is None or fresh["age_s"] > FEED_MAX_AGE_S else self.measured_tip()
                err = float(np.linalg.norm(tip[[0, 2]] - target)) if tip is not None else None
                why = "" if closed_loop is False else "camera feed not running, "
                st.update(state="done", error_mm=err,
                          message=note + f"{why}open loop (model only, typically ±{model.info['cv_error_mm']['median']:.0f} mm)")
                return
            d = np.array([cur["A"], cur["B"]])
            for it in range(1, MAX_ITERATIONS + 1):
                st["iteration"] = it
                time.sleep(0.3)  # let the arm stop swinging
                tip = self.measured_tip()
                if tip is None:
                    st.update(state="done", message=note + "tip not visible to the camera; stopped (open loop result)")
                    return
                err = target - tip[[0, 2]]
                st["error_mm"] = float(np.linalg.norm(err))
                st["message"] = note + f"correcting (iteration {it})"
                if st["error_mm"] <= TOLERANCE_MM:
                    st.update(state="done", message=note + f"reached within {TOLERANCE_MM:.0f} mm")
                    return
                J = model.jacobian_xz(d)
                step = np.linalg.solve(J.T @ J + 1e-3 * np.eye(2), J.T @ err)
                n = np.linalg.norm(step)
                if n > MAX_CORRECTION:
                    step *= MAX_CORRECTION / n
                d_new = clip_disk(d + step)
                if np.linalg.norm(d_new - d) < 1e-3:
                    st.update(state="done", message=note + "at the edge of the workspace; can't get closer")
                    return
                cur, lim = self._move({"A": float(d[0]), "B": float(d[1])}, {"A": float(d_new[0]), "B": float(d_new[1])})
                d = np.array([cur["A"], cur["B"]])
                if lim:
                    st.update(state="limited", message=note + f"force limit ({lim}); stopped short")
                    return
            st.update(state="done", message=note + f"best after {MAX_ITERATIONS} corrections")
        except Stopped:
            st.update(state="stopped", message="stopped")
            self._hold()
        except Fail as e:
            st.update(state="failed", message=f"{e}; released motors to rest")
            self._release_all()
        except Exception as e:  # e.g. lost connection
            st.update(state="failed", message=str(e))

    def _hold(self):
        for i in motors.IDS:
            m = self.ctl.state.get(i)
            if m and m["torque"]:
                try:
                    self.ctl.bus.set_goal(i, m["pulse"])
                    self.ctl.commanded[i] = (m["pulse"], time.monotonic())
                except Exception:
                    pass

    def snapshot(self):
        model = self.model
        out = {"model": model is not None, "status": self.status, "busy": self.busy}
        feed = self.latest_feed()
        out["feed"] = feed if feed and feed["age_s"] <= 2.0 else None
        if model is not None and self.ctl.bus and all(self.ctl.state.get(i) for i in motors.IDS) \
                and all(m["zero_rad"] is not None for m in self.ctl.settings["motors"]):
            d = self.pose()
            out["pose"] = d
            out["predicted"] = model.predict([[d["A"], d["B"]]])[0].tolist()
        return out

    def model_info(self):
        model = self.model
        if model is None:
            return None
        i = model.info
        return {"base": i["base"], "length_mm": i["length_mm"], "rest_tip": i["rest_tip"], "samples": i["samples"],
                "cv_error_mm": i["cv_error_mm"], "n": i["n"], "fitted": i["fitted"],
                "outline": model.outline_xz.round(1).tolist()}
