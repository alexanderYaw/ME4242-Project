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
TICK_S = 0.05            # goal update interval while streaming a move
SPEED_MARGIN = 0.85      # stream at most this fraction of the motors' speed limit
MIN_SEGMENT_S = 0.4      # shortest streamed move
BACKOFF_S = 0.4          # on a force limit, go back to where the goals were this long ago
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


class TipController:
    def __init__(self, ctl, model_path):
        self.ctl = ctl
        self.model_path = Path(model_path)
        self._model, self._model_mtime = None, None
        self.feed = collections.deque(maxlen=200)  # (monotonic time, {x, y, z, markers})
        self.status = {"state": "idle", "target": None, "iteration": 0, "error_mm": None, "message": ""}
        self._thread = None
        self._stop = threading.Event()
        self._motion = threading.Lock()  # one move at a time (tip goto or move_bend)
        self.soft_ma, self.hard_ma = SOFT_MA, HARD_MA

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

    # -- motion ---------------------------------------------------------------------
    def _goal(self, i, rad):
        lo, hi = self.ctl.limits_rad(i)
        pulse = motors.to_pulse(min(max(rad, lo), hi))
        self.ctl.bus.set_goal(i, pulse)
        self.ctl.commanded[i] = (pulse, time.monotonic())

    def _check(self, soft, pulling=None):
        """Safety checks on the latest motor state; raises Stopped / Fail / ForceLimit. The soft current
        limit only applies to motors in `pulling` (all if None): a motor releasing a tense cable must never
        be stopped from releasing. The hard limit applies to every motor."""
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
            if ma > self.hard_ma:
                raise Fail(f"motor {i} current {ma:.0f} mA > {self.hard_ma:.0f} mA")
            if soft and ma > self.soft_ma and (pulling is None or i in pulling):
                raise ForceLimit(i, ma)
        return st

    def _wait_settled(self, soft=True, timeout=8.0, window=8, pulling=None):
        t0, hist = time.monotonic(), []
        while True:
            st = self._check(soft, pulling)
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

    def _current_goals(self):
        return {i: motors.to_angle(self.ctl.state[i]["goal_pulse"]) for i in motors.IDS}

    def _stream(self, start, end):
        """Stream goals smoothly from `start` to `end` (absolute angles per motor, rad), easing in and out,
        then wait for the arm to stop. Speed follows the GUI's speed setting. If a pulling motor passes the
        soft current limit, back off to where the motors actually were BACKOFF_S earlier and return the
        ForceLimit; otherwise return None."""
        speed = self.ctl.settings["profile_velocity"] * motors.VELOCITY_UNIT_RPM * 6  # deg/s
        span = max(abs(math.degrees(end[i] - start[i])) for i in motors.IDS)
        duration = max(MIN_SEGMENT_S, 1.5 * span / (SPEED_MARGIN * speed))  # smoothstep peaks at 1.5x mean
        pulling = {i for i in motors.IDS if end[i] > start[i] + math.radians(0.5)}
        seen, t0 = [], time.monotonic()  # (time, measured absolute angles)
        try:
            while True:
                u = min(1.0, (time.monotonic() - t0) / duration)
                k = u * u * (3 - 2 * u)
                for i in motors.IDS:
                    self._goal(i, start[i] + (end[i] - start[i]) * k)
                st = self._check(soft=True, pulling=pulling)
                seen.append((time.monotonic(), {i: motors.to_angle(st[i]["pulse"]) for i in motors.IDS}))
                if u >= 1:
                    break
                time.sleep(TICK_S)
            self._wait_settled(pulling=pulling)
            return None
        except ForceLimit as lim:
            now = time.monotonic()
            back = next((a for t, a in reversed(seen) if now - t >= BACKOFF_S), start)
            for i, g in back.items():
                self._goal(i, g)
            self._wait_settled(soft=False)
            return lim

    def _move(self, cur, tgt):
        """Move smoothly to bend values tgt. A pair that changes sign first releases its pulling cable (the
        only stop on the way) before the opposite cable pulls. Returns (pose, ForceLimit or None)."""
        stops = []
        if any(cur[p] * tgt[p] < 0 for p in PAIRS):
            stops.append({p: (0.0 if cur[p] * tgt[p] < 0 else cur[p]) for p in PAIRS})
        stops.append(dict(tgt))
        start = self._current_goals()
        for stop in stops:
            end = self.targets_abs(stop)
            lim = self._stream(start, end)
            if lim:
                return self.pose(), lim
            start = end
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
        return self._motion.locked()

    def check_ready(self, need_model=True):
        if not self.ctl.bus:
            raise ValueError("Not connected")
        if need_model and self.model is None:
            raise ValueError(f"No tip model at {self.model_path}; run fit_tip_model.py first")
        if any(m["zero_rad"] is None for m in self.ctl.settings["motors"]):
            raise ValueError("Calibrate every motor's taut zero first")
        if any(self.ctl.state.get(i) is None for i in motors.IDS):
            raise ValueError("Not all motors are responding")
        if any(self.ctl.state[i]["hw_error"] for i in motors.IDS):
            raise ValueError("A motor has a hardware error; reboot it first")
        if self.busy:
            raise ValueError("Already moving; press Stop first")

    def _prepare(self):
        """Torque on (holding position) with the speed and acceleration profile applied."""
        self.ctl.apply_motion_profile(self.ctl.bus)
        for i in motors.IDS:
            if not self.ctl.state[i]["torque"]:
                self.ctl.bus.set_torque(i, True)
                self.ctl.state[i] = {**self.ctl.state[i], "torque": True}
                self.ctl.commanded.pop(i, None)

    def goto(self, x, z, closed_loop=True):
        self.check_ready()
        if not self._motion.acquire(blocking=False):
            raise ValueError("Already moving; press Stop first")
        self._stop.clear()
        self.status = {"state": "moving", "target": [x, z], "iteration": 0, "error_mm": None, "message": "starting"}

        def run():
            try:
                self._run(x, z, closed_loop)
            finally:
                self._motion.release()
        threading.Thread(target=run, daemon=True).start()

    def move_bend(self, a, b, soft_ma=None, hard_ma=None):
        """Blocking smooth move to bend values (a, b), used by collect_tip_data.py.
        Returns {"pose", "limited"}; raises ValueError (not ready / busy / stopped) or RuntimeError (fault)."""
        self.check_ready(need_model=False)
        if math.hypot(a, b) > 1 + 1e-9:
            raise ValueError("bend values must satisfy dA^2 + dB^2 <= 1")
        if not self._motion.acquire(blocking=False):
            raise ValueError("Already moving; press Stop first")
        self._stop.clear()
        self.soft_ma, self.hard_ma = soft_ma or SOFT_MA, hard_ma or HARD_MA
        try:
            self._prepare()
            pose, lim = self._move(self.pose(), {"A": a, "B": b})
            return {"pose": pose, "limited": {"motor": lim.motor, "ma": lim.ma} if lim else None}
        except Stopped:
            self._hold()
            raise ValueError("stopped")
        except Fail as e:
            self._release_all()
            raise RuntimeError(f"{e}; released motors to rest")
        finally:
            self.soft_ma, self.hard_ma = SOFT_MA, HARD_MA
            self._motion.release()

    def stop(self):
        self._stop.set()

    def _run(self, x, z, closed_loop):
        st = self.status
        try:
            self._prepare()
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
