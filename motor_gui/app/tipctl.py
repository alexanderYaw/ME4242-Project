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
BACKOFF_S = 0.4          # on a force limit, go back to where the motors were this long ago
BACKOFF_GRACE_S = 0.5    # ... and give the current this long to drop before the hard limit applies again
SOFT_MA = 1000           # above this: back off and stop short
HARD_MA = 1100           # above this: release all motors and abort
MA_PER_UNIT = 2.69
MAX_ERROR_DEG = 8.0      # motor settled further than this from its goal = blocked
FEED_MAX_AGE_S = 0.5
TOLERANCE_MM = 3.0
MAX_ITERATIONS = 8
MAX_CORRECTION = 0.15    # largest change in d per closed-loop correction
OUTSIDE_MM = 15.0        # target further than this from the model's reach = outside the workspace
JOY_TICK_S = 0.05        # gamepad follow loop interval
JOY_STALE_S = 0.3        # gamepad input older than this = hold (tab hidden, gamepad unplugged, ...)
JOY_WALL_CLEAR = 0.02    # a force-limit wall clears as soon as the target eases off this far (bend units)
JOY_WALL_RETRY_S = 2.0   # ... or after this long, to re-try the limit (it may have been a brief spike)
JOY_BACKOFF_S = 0.15     # gamepad: on a force limit, go back to where the motors were this long ago
JOY_POLAR_MIN_R = 0.1    # below this bend, follow the stick in a straight line (direction is ill-defined)
JOY_FLICK_JUMP = math.radians(90)  # stick direction jumping this much between two inputs = flick across
PAYOUT_RATIO = 1.0       # taut mode: the opposite motor unwinds this many degrees per degree its partner winds in
PAYOUT_GUARD_MA = 300    # taut mode: a paying-out motor above this current unwinds further on its own ...
PAYOUT_RELIEF_STEP = math.radians(1.0)  # ... this much per tick
TAUT_RAMP_S = 0.5        # taut mode: cables are tightened to taut over this long, while the trigger is held
JOY_LEAD = 0.1           # jog: how far the target may run ahead of the arm (bend units) ...
JOY_LEAD_MM = 15.0       # ... or, in tip mode, in mm; so the arm stops where it is when you let go
ARC_POINTS = 16          # points along the arm checked against the obstacle
PLAN_STEP = 0.025        # obstacle planner: grid of bend values
CHECK_STEP = 0.01        # ... path segments are checked this often (bend units)
NEAR_PENALTY = 20.0      # ... extra cost for moving inside the clearance margin (times the missing fraction)
CLEARANCE_MM = 20.0      # default: the arm's centreline stays this far from the rod's surface
SCENE_GRID = 41          # 3D view: tip surface sampled on this many bend values per pair
PLAN_ARMS = 9            # 3D view: arm shapes shown along a planned path


class Stopped(Exception):
    pass


class Fail(Exception):
    pass


class NoPath(Exception):
    """The obstacle planner found no clear path (nothing has moved)."""


class ForceLimit(Exception):
    def __init__(self, motor, ma):
        super().__init__(f"motor {motor} at {ma:.0f} mA")
        self.motor, self.ma = motor, ma


class TipModel:
    def __init__(self, path):
        m = json.loads(Path(path).read_text())
        self.info = m
        self.deg = m["degree"]
        self.form = m.get("type", "quadrant_poly")
        self.max_bend = None
        if self.form == "cc":
            self.cc = m["coeffs"]  # see fit_tip_model.py: constant-curvature arm
            self.max_bend = m.get("max_bend_rad")
        elif self.form == "per_motor":
            self.W = np.array(m["coeffs"])  # see fit_tip_model.py: terms 1, a+, a-, b+, b-, a+^2, a-^2, b+^2, b-^2
        else:
            self.coeffs = {q: np.array(c) for q, c in m["coeffs"].items()}
        g = np.linspace(-1, 1, 161)
        A, B = np.meshgrid(g, g)
        D = np.column_stack([A.ravel(), B.ravel()])
        # workspace: the unit square of bend values (each pair independently up to its full pull), and for the
        # constant-curvature model only bends below max_bend (beyond it the tip curls back inward)
        self.grid_d = D if self.max_bend is None else D[self.bend_rad(D) <= self.max_bend]
        self.grid_xz = self.predict(self.grid_d)[:, [0, 2]]
        t = np.linspace(0, 2 * math.pi, 361)
        circle = np.column_stack([np.cos(t), np.sin(t)])
        edge = circle / np.max(np.abs(circle), axis=1, keepdims=True)  # same directions, on the square
        edge = np.array([self.clip(e) for e in edge])
        self.outline_xz = self.predict(edge)[:, [0, 2]]
        self.rest_xz = self.predict([[0.0, 0.0]])[0, [0, 2]]
        # direction (deg, from +x toward +z) each motor 0..3 bends the tip; the GUI orients its top view with it
        moved = self.predict([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]])[:, [0, 2]] - self.rest_xz
        self.motor_dirs_deg = np.degrees(np.arctan2(moved[:, 1], moved[:, 0]))
        rel = self.outline_xz - self.rest_xz
        order = np.argsort(np.arctan2(rel[:, 1], rel[:, 0]))
        self._reach_ang = np.arctan2(rel[:, 1], rel[:, 0])[order]
        self._reach_r = np.hypot(rel[:, 0], rel[:, 1])[order]
        # typical tip travel for a full unit of bend (sets the tip-mode jog speed)
        self.mm_per_bend = float(np.mean(np.linalg.norm(self.predict(circle)[:, [0, 2]] - self.rest_xz, axis=1)))

    def reach_mm(self, angle):
        """Distance from the rest tip to the edge of the reachable area, in direction `angle` (top view)."""
        a = np.concatenate([self._reach_ang - 2 * np.pi, self._reach_ang, self._reach_ang + 2 * np.pi])
        r = np.tile(self._reach_r, 3)
        return float(np.interp(angle, a, r))

    def _features(self, D):
        a, b = D[:, 0], D[:, 1]
        return np.column_stack([a ** i * b ** j for i in range(self.deg + 1) for j in range(self.deg + 1 - i)])

    def _cc_bend(self, D):
        a, b = D[:, 0], D[:, 1]
        pulls = np.column_stack([np.maximum(a, 0), np.maximum(b, 0), np.maximum(-a, 0), np.maximum(-b, 0)])
        beta = np.tile(np.array(self.cc["bend0"], dtype=float), (len(D), 1))
        for i, m in enumerate(self.cc["motors"]):
            g = m["c1"] * pulls[:, i] + m["c2"] * pulls[:, i] ** 2
            beta += g[:, None] * np.array([math.cos(m["phi"]), math.sin(m["phi"])])
        return beta

    def bend_rad(self, D):
        """Total bend angle of the arc (constant-curvature model only)."""
        beta = self._cc_bend(np.atleast_2d(np.asarray(D, dtype=float)))
        return np.hypot(beta[:, 0], beta[:, 1])

    def clip(self, d):
        """Nearest bend values (along the same direction) inside the workspace."""
        d = np.clip(np.asarray(d, dtype=float), -1.0, 1.0)
        if self.max_bend is None or self.bend_rad(d)[0] <= self.max_bend:
            return d
        lo, hi = 0.0, 1.0  # bend grows along the ray from the centre: bisect for the limit
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if self.bend_rad(d * mid)[0] <= self.max_bend else (lo, mid)
        return d * lo

    def predict(self, D):
        D = np.atleast_2d(np.asarray(D, dtype=float))
        if self.form == "cc":
            beta = self._cc_bend(D)
            th = np.hypot(beta[:, 0], beta[:, 1])
            L, (bx, by, bz) = self.cc["length_mm"], self.cc["base"]
            small = th < 1e-6
            ths = np.where(small, 1.0, th)
            h = np.where(small, L * th / 2, L * (1 - np.cos(ths)) / ths)  # horizontal reach
            v = np.where(small, L, L * np.sin(ths) / ths)                 # depth below the attachment
            ux, uz = np.where(small, 1.0, beta[:, 0] / ths), np.where(small, 0.0, beta[:, 1] / ths)
            return np.column_stack([bx + h * ux, by - v, bz + h * uz])
        if self.form == "per_motor":
            a, b = D[:, 0], D[:, 1]
            ap, an, bp, bn = np.maximum(a, 0), np.minimum(a, 0), np.maximum(b, 0), np.minimum(b, 0)
            X = np.column_stack([np.ones_like(a), ap, an, bp, bn, ap ** 2, an ** 2, bp ** 2, bn ** 2])
            return X @ self.W
        out = np.zeros((len(D), 3))
        for q, (sa, sb) in QUADRANTS.items():
            m = (D[:, 0] * sa >= -1e-12) & (D[:, 1] * sb >= -1e-12)
            if m.any():
                out[m] = self._features(D[m]) @ self.coeffs[q]
        return out

    @property
    def has_shape(self):
        return self.form == "cc"

    def backbone(self, D, n=ARC_POINTS):
        """Points along the arm (constant-curvature model) from just below the attachment to the tip,
        shape (len(D), n, 3)."""
        D = np.atleast_2d(np.asarray(D, dtype=float))
        beta = self._cc_bend(D)
        th = np.hypot(beta[:, 0], beta[:, 1])
        L, base = self.cc["length_mm"], np.array(self.cc["base"], dtype=float)
        s = np.arange(1, n + 1) / n                   # fraction of the arm's length
        a = th[:, None] * s                           # bend reached at each point
        small = a < 1e-6
        a_ = np.where(small, 1.0, a)
        h = np.where(small, L * s * a / 2, L * s * (1 - np.cos(a_)) / a_)
        v = np.where(small, L * s, L * s * np.sin(a_) / a_)
        straight = th < 1e-6
        ux = np.where(straight, 1.0, beta[:, 0] / np.where(straight, 1.0, th))[:, None]
        uz = np.where(straight, 0.0, beta[:, 1] / np.where(straight, 1.0, th))[:, None]
        return np.stack([base[0] + h * ux, base[1] - v,base[2] + h * uz], axis=-1)

    def jacobian_xz(self, d, h=0.02):
        J = np.zeros((2, 2))
        for k in range(2):
            e = np.zeros(2)
            e[k] = h
            J[:, k] = (self.predict(d + e)[0, [0, 2]] - self.predict(d - e)[0, [0, 2]]) / (2 * h)
        return J

    def _edge_normals(self, d):
        """Outward normals (in bend values) of the workspace edges d is on."""
        out = []
        for k in range(2):
            if abs(d[k]) >= 1 - 1e-6:
                n = np.zeros(2)
                n[k] = math.copysign(1.0, d[k])
                out.append(n)
        if self.max_bend is not None and self.bend_rad(d)[0] >= self.max_bend - 1e-3:
            h = 1e-4
            g = np.array([(self.bend_rad(d + e)[0] - self.bend_rad(d - e)[0]) / (2 * h) for e in np.eye(2) * h])
            if np.linalg.norm(g) > 1e-9:
                out.append(g / np.linalg.norm(g))
        return out

    def step_toward(self, d, err):
        """Damped Gauss-Newton step in bend values that moves the predicted tip by err (mm, x/z). At the edge of the
        workspace, directions pointing out through it are dropped and the step is re-solved along the edge, so the
        tip slides only as far as it gets closer to the target (instead of drifting along the edge)."""
        J = self.jacobian_xz(d)
        step = np.linalg.solve(J.T @ J + 1e-3 * np.eye(2), J.T @ err)
        normals = self._edge_normals(d)
        if not any(n @ step > 0 for n in normals):
            return step
        best, best_gain = np.zeros(2), 0.0
        for n in normals:  # slide along one edge, if that doesn't push out through another
            t = np.array([-n[1], n[0]])
            Jt = J @ t
            cand = t * (Jt @ err) / (Jt @ Jt + 1e-3)
            gain = (J @ cand) @ err
            if gain > best_gain and not any(m @ cand > 1e-12 for m in normals):
                best, best_gain = cand, gain
        return best

    def inverse(self, x, z):
        """Bend values whose predicted tip is closest to (x, z). Returns (d, predicted xz, distance mm)."""
        t = np.array([x, z])
        d = self.grid_d[np.argmin(np.linalg.norm(self.grid_xz - t, axis=1))].copy()
        for _ in range(10):  # refine with damped Gauss-Newton, staying inside the workspace
            step = self.step_toward(d, t - self.predict(d)[0, [0, 2]])
            d = self.clip(d + np.clip(step, -0.05, 0.05))
        p = self.predict(d)[0, [0, 2]]
        return d, p, float(np.linalg.norm(p - t))

    def inverse_near(self, x, z, d0, iters=5, max_step=0.02):
        """Like inverse(), but only searches near d0 (damped Gauss-Newton from d0, each step at most max_step),
        so the result moves continuously with the target instead of jumping to another solution of the model.
        At the edge of the reachable area it slides along the boundary. Returns (d, predicted xz, distance mm)."""
        t = np.array([x, z])
        d = self.clip(np.array(d0, dtype=float))
        for _ in range(iters):
            step = self.step_toward(d, t - self.predict(d)[0, [0, 2]])
            d = self.clip(d + np.clip(step, -max_step, max_step))
        p = self.predict(d)[0, [0, 2]]
        return d, p, float(np.linalg.norm(p - t))


def clamp_bend(d):
    """Keep bend values in the unit square: each pair independently up to its full pull."""
    return {p: max(-1.0, min(1.0, v)) for p, v in d.items()}


def rod_clearance(P, rod):
    """Distance (mm) from points P (..., 3) to the surface of the rod (a segment p0-p1 with a radius)."""
    a, b = np.asarray(rod["p0"], dtype=float), np.asarray(rod["p1"], dtype=float)
    ab = b - a
    t = np.clip((P - a) @ ab / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(P - (a + t[..., None] * ab), axis=-1) - rod["radius"]


def arc_through_tip(base, tip, n=ARC_POINTS):
    """The arm's shape from the measured tip alone: the constant-curvature arc that leaves the attachment straight
    down and ends at the tip (bend = 2 * atan(horizontal / vertical offset)). Points (n, 3), as backbone()."""
    base, tip = np.asarray(base, dtype=float), np.asarray(tip, dtype=float)
    hz = tip[[0, 2]] - base[[0, 2]]
    h, v = float(np.linalg.norm(hz)), base[1] - tip[1]
    s = np.arange(1, n + 1) / n
    if h < 1e-3:
        return base + np.outer(s, [0.0, -v, 0.0])
    th, R = 2 * math.atan2(h, v), (h * h + v * v) / (2 * h)
    u = hz / h
    hh, vv = R * (1 - np.cos(th * s)), R * np.sin(th * s)
    return np.column_stack([base[0] + hh * u[0], base[1] - vv, base[2] + hh * u[1]])


def split_at_axes(start, path):
    """Insert the points where a straight path crosses dA = 0 or dB = 0, so every piece stays in one quadrant:
    there a pair's pulling cable is fully released before the opposite one pulls (the pair rule), and motor
    angles change linearly with the bend values."""
    out, prev = [], np.asarray(start, dtype=float)
    for p in path:
        p = np.asarray(p, dtype=float)
        cross = sorted((prev[k] / (prev[k] - p[k]), k) for k in range(2) if prev[k] * p[k] < 0)
        for t, k in cross:
            q = prev + t * (p - prev)
            q[k] = 0.0
            out.append(q)
        out.append(p)
        prev = p
    return out


def densify(d0, path, k=5):
    """d0 and points every 1/k of each straight piece of the path (bend values), shape (N, 2)."""
    d0 = np.asarray(d0, dtype=float)
    pts = [d0] + [np.asarray(p, dtype=float) for p in path]
    return np.array([d0] + [a + (b - a) * t for a, b in zip(pts, pts[1:]) for t in np.arange(1, k + 1) / k])


class Planner:
    """Paths in bend values that keep the arm (constant-curvature model: the whole arc, not just the tip) clear of
    the rod.

    Every point of a bend-value grid over the full motor range gets the arm's clearance from the rod (so a path
    may curl the arm past the tip-mode bend limit, e.g. to lift it over the rod). A path is the cheapest grid route
    (Dijkstra, 8-connected), then shortened wherever a straight line is no worse. Inside the clearance margin
    moves cost extra, and an arm that starts inside it may not get any closer, so it first backs away."""

    def __init__(self, model, rod, need):
        self.model, self.rod, self.need = model, rod, float(need)
        self.g = np.linspace(-1, 1, int(round(2 / PLAN_STEP)) + 1)
        n = len(self.g)
        A, B = np.meshgrid(self.g, self.g, indexing="ij")
        self.D = np.stack([A, B], axis=-1)
        self.clear = self.clearance(self.D.reshape(-1, 2)).reshape(n, n)

    def clearance(self, D):
        return np.min(rod_clearance(self.model.backbone(np.atleast_2d(D)), self.rod), axis=1)

    def weight(self, c, floor):
        """Cost per bend unit at clearance c; infinite closer than floor."""
        c = np.asarray(c, dtype=float)
        w = 1 + NEAR_PENALTY * np.clip((self.need - c) / self.need, 0, None)
        return np.where(c < floor, np.inf, w)

    def seg_cost(self, a, b, floor):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        length = float(np.linalg.norm(b - a))
        if length < 1e-9:
            return 0.0
        k = max(1, math.ceil(length / CHECK_STEP))
        w = self.weight(self.clearance(a + np.outer(np.arange(k + 1) / k, b - a)), floor)
        return length * float(np.mean((w[1:] + w[:-1]) / 2))

    def nearest_clear(self, d):
        """Grid point with a clear arm whose tip is closest (top view) to the tip at d."""
        ok = self.clear >= self.need
        if not ok.any():
            return None
        D = self.D[ok]
        tips = self.model.predict(D)[:, [0, 2]]
        return D[np.argmin(np.linalg.norm(tips - self.model.predict(d)[0, [0, 2]], axis=1))]

    def plan(self, d0, d1):
        """Waypoints (bend values, excluding d0) to d1 or, if the arm would be too close to the rod there, to the
        clear point with the nearest tip. Returns (waypoints, goal used, note); raises NoPath if there is no way."""
        d0, d1 = np.asarray(d0, dtype=float), np.asarray(d1, dtype=float)
        c0 = float(self.clearance(d0)[0])
        floor = min(self.need, c0 - 1.0)
        note = ""
        if float(self.clearance(d1)[0]) < self.need:
            alt = self.nearest_clear(d1)
            if alt is None:
                raise NoPath(f"the arm can't get {self.need:.0f} mm clear of the obstacle anywhere")
            d1 = alt
            note = "target too close to the obstacle; going to the nearest clear point. "
        if self.seg_cost(d0, d1, floor) <= np.linalg.norm(d1 - d0) * (1 + 1e-6):
            return [d1], d1, note  # straight there, clear all the way
        route = self._dijkstra(d0, d1, floor)
        if route is None:
            gap = self._widest_gap(d0, d1)
            raise NoPath(f"no path keeps the arm {self.need:.0f} mm clear of the obstacle" + (
                f"; the widest way passes {gap:.0f} mm from it (lower the clearance to below that to allow it)"
                if gap > 0 else "; every way touches it"))
        pts = self._shorten([d0] + route + [d1], floor)
        if len(pts) > 2:
            note += f"going around the obstacle ({len(pts) - 2} waypoints). "
        return pts[1:], d1, note

    def _corners(self, d):
        """The grid points around d."""
        n = len(self.g)
        f = (np.asarray(d) + 1) / PLAN_STEP
        lo = np.clip(np.floor(f).astype(int), 0, n - 1)
        return {(min(lo[0] + i, n - 1), min(lo[1] + j, n - 1)) for i in (0, 1) for j in (0, 1)}

    def _dijkstra(self, d0, d1, floor):
        import heapq
        n = len(self.g)
        W = self.weight(self.clear, floor)
        dist, prev, heap = {}, {}, []
        for c in self._corners(d0):
            cost = self.seg_cost(d0, self.D[c], floor)
            if np.isfinite(cost) and cost < dist.get(c, np.inf):
                dist[c], prev[c] = cost, None
                heapq.heappush(heap, (cost, c))
        goals = {c: self.seg_cost(self.D[c], d1, floor) for c in self._corners(d1)}
        best, best_node = np.inf, None
        done = set()
        while heap:
            du, u = heapq.heappop(heap)
            if u in done or du > dist.get(u, np.inf):
                continue
            if du >= best:
                break
            done.add(u)
            if u in goals and du + goals[u] < best:
                best, best_node = du + goals[u], u
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    v = (u[0] + di, u[1] + dj)
                    if (di or dj) and 0 <= v[0] < n and 0 <= v[1] < n and np.isfinite(W[v]):
                        nd = du + PLAN_STEP * math.hypot(di, dj) * (W[u] + W[v]) / 2
                        if nd < dist.get(v, np.inf):
                            dist[v], prev[v] = nd, u
                            heapq.heappush(heap, (nd, v))
        if best_node is None:
            return None
        route, u = [], best_node
        while u is not None:
            route.append(self.D[u])
            u = prev[u]
        return route[::-1]

    def _widest_gap(self, d0, d1):
        """Largest clearance a grid path from d0 to d1 can keep all the way (widest-path search)."""
        import heapq
        n = len(self.g)
        best, heap = {}, []
        for c in self._corners(d0):
            best[c] = float(self.clear[c])
            heapq.heappush(heap, (-best[c], c))
        goals = self._corners(d1)
        while heap:
            w, u = heapq.heappop(heap)
            if -w < best.get(u, -np.inf):
                continue
            if u in goals:
                return -w
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    v = (u[0] + di, u[1] + dj)
                    if (di or dj) and 0 <= v[0] < n and 0 <= v[1] < n:
                        nw = min(-w, float(self.clear[v]))
                        if nw > best.get(v, -np.inf):
                            best[v] = nw
                            heapq.heappush(heap, (-nw, v))
        return -np.inf

    def _shorten(self, pts, floor):
        """Replace runs of waypoints by a straight line wherever that costs no more."""
        step = [self.seg_cost(a, b, floor) for a, b in zip(pts, pts[1:])]
        cum = np.concatenate([[0.0], np.cumsum(step)])
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and self.seg_cost(pts[i], pts[j], floor) > (cum[j] - cum[i]) * 1.02 + 1e-9:
                j -= 1
            out.append(pts[j])
            i = j
        return out



class TipController:
    def __init__(self, ctl, model_path):
        self.ctl = ctl
        self.model_path = Path(model_path)
        self._model, self._model_mtime = None, None
        self.feed = collections.deque(maxlen=200)  # (monotonic time, {x, y, z, markers})
        self.status = {"state": "idle", "target": None, "iteration": 0, "error_mm": None, "message": ""}
        self._thread = None
        self._stop = threading.Event()
        self._motion = threading.Lock()  # one move at a time (tip goto, move_bend or gamepad)
        self.soft_ma, self.hard_ma = SOFT_MA, HARD_MA
        self._hard_grace_until = 0.0  # after a back-off, the hard limit waits this long for the current to drop
        self.joy = {"active": False, "message": ""}
        # obstacle (rod, marked in the GUI) and the avoidance settings; kept in obstacle.json next to the model
        self.obstacle_path = self.model_path.parent / "obstacle.json"
        self.obs = {"avoid": True, "clearance_mm": CLEARANCE_MM, "hsv": None, "rod": None, "rod_t": None,
                    "source": None}
        try:
            self.obs.update(json.loads(self.obstacle_path.read_text()))
        except (OSError, ValueError):
            pass
        if self.obs["source"] == "colour":  # from an older tip_feed.py: colour-detected rods aren't avoided now
            self.obs.update(rod=None, rod_t=None, source=None)
        # rods and other objects found by colour by tip_feed.py: shown in the GUI only, not avoided
        self.rods = {"list": [], "t": None, "version": 0}
        self.objects = {"list": [], "version": 0}
        self.obs_version = 0
        self.obs_feed = {"t": None, "seen": False, "pick": None, "mark": None, "reset": False, "error": ""}
        self._planner = (None, None)
        self.plan_xz = None  # predicted tip along the current planned path, for the GUI
        self.plan_d, self.plan_id = None, 0  # ... and its bend values (3D view: arm shapes along it)
        self.preview = None  # planned path to a target without moving (3D view)

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

    # -- obstacle -------------------------------------------------------------------
    def _save_obs(self):
        try:
            self.obstacle_path.write_text(json.dumps(self.obs, indent=1))
        except OSError:
            pass

    def _set_rod(self, rod, source):
        old = self.obs["rod"]
        rod = {"p0": [float(v) for v in rod["p0"]], "p1": [float(v) for v in rod["p1"]],
               "radius": float(rod["radius"]), **{k: rod[k] for k in ("open", "visible_mm") if k in rod}}
        moved = old is None or self.obs["source"] != source or abs(old["radius"] - rod["radius"]) > 1.0 or \
            max(np.linalg.norm(np.subtract(old[k], rod[k])) for k in ("p0", "p1")) > 2.0
        self.obs.update(rod_t=time.time(), source=source)
        if moved:  # small jitter doesn't re-plan or redraw
            self.obs["rod"] = rod
            self.obs_version += 1
            self._save_obs()

    def _set_rods(self, rods):
        old = self.rods["list"]
        rods = [{"p0": [float(v) for v in r["p0"]], "p1": [float(v) for v in r["p1"]], "radius": float(r["radius"]),
                 "colour": str(r.get("colour", "")), **{k: r[k] for k in ("open", "visible_mm") if k in r}}
                for r in rods]
        if rods:
            self.rods["t"] = time.time()
        moved = len(rods) != len(old) or any(
            a["colour"] != b["colour"] or abs(a["radius"] - b["radius"]) > 1.0 or
            max(np.linalg.norm(np.subtract(a[k], b[k])) for k in ("p0", "p1")) > 2.0 for a, b in zip(old, rods))
        if moved:  # small jitter doesn't redraw
            self.rods["list"] = rods
            self.rods["version"] += 1

    def _set_objects(self, objects):
        self.objects = {"list": objects, "version": self.objects["version"] + 1}

    def objects_info(self):
        """Objects with their surface meshes (3D view)."""
        return {"version": self.objects["version"], "objects": self.objects["list"]}

    def obstacle_feed(self, rods=None, seen=False, hsv_picked=None, marked=None, error=None, objects=None):
        """Update from tip_feed.py (a few times per second). Returns what the feed needs: the rod colour, a
        pending colour pick or end marking (image fractions), and whether to forget its tracked rods."""
        f = self.obs_feed
        f.update(t=time.monotonic(), seen=bool(seen))
        if error:
            f.update(error=error, pick=None, mark=None)
        if hsv_picked:
            self.obs["hsv"] = hsv_picked
            f.update(pick=None, error="", reset=True)
            self._save_obs()
        elif marked:
            f.update(mark=None, error="")
            self._set_rod(marked, "marked")
        if rods is not None:
            self._set_rods(rods)
        if objects is not None and objects != self.objects["list"]:
            self._set_objects(objects)
        out = {"hsv": self.obs["hsv"], "pick": f["pick"], "mark": f["mark"], "reset": f["reset"]}
        f["reset"] = False
        return out

    def obstacle_pick(self, u, v):
        self.obs_feed.update(pick=[u, v], mark=None, error="")

    def obstacle_mark(self, points, radius):
        self.obs_feed.update(mark={"points": points, "radius": radius}, pick=None, error="")

    def obstacle_clear(self):
        """Forget the marked rod, the detected rods and a picked colour (back to tip_feed.py's colours)."""
        self.obs.update(rod=None, rod_t=None, source=None, hsv=None)
        self.obs_feed.update(reset=True, pick=None, mark=None, error="")
        self.obs_version += 1
        self._set_rods([])
        self._set_objects([])
        self._save_obs()

    def obstacle_settings(self, avoid=None, clearance_mm=None):
        if avoid is not None:
            self.obs["avoid"] = bool(avoid)
        if clearance_mm is not None:
            self.obs["clearance_mm"] = float(clearance_mm)
            self.obs_version += 1
        self._save_obs()

    def planner(self):
        """Obstacle planner for the current rod and clearance, or None (no rod, avoidance off, or a model without
        the arm's shape)."""
        model = self.model
        if not self.obs["avoid"] or self.obs["rod"] is None or model is None or not model.has_shape:
            return None
        key = (self._model_mtime, self.obs_version)
        if self._planner[0] != key:
            self._planner = (key, Planner(model, self.obs["rod"], self.obs["clearance_mm"]))
        return self._planner[1]

    def obstacle_map(self):
        """For the GUI: the rod and the tip positions (inside the tip-mode workspace) where the arm would be closer
        to it than the clearance (near) or touch it (hit)."""
        p = self.planner()
        out = {"version": self.obs_version, "rod": self.obs["rod"], "near": [], "hit": []}
        if p is None:
            return out
        D, c = p.D[::2, ::2].reshape(-1, 2), p.clear[::2, ::2].ravel()
        model = self.model
        inside = model.bend_rad(D) <= model.max_bend if model.max_bend is not None else np.ones(len(D), bool)
        xz = model.predict(D)[:, [0, 2]].round(1)
        out["near"] = xz[inside & (c >= 0) & (c < p.need)].tolist()
        out["hit"] = xz[inside & (c < 0)].tolist()
        out["step_mm"] = round(2 * PLAN_STEP * model.mm_per_bend, 1)
        return out

    def _obstacle_snapshot(self, d):
        o, f = self.obs, self.obs_feed
        out = {"rod": o["rod"], "source": o["source"], "avoid": o["avoid"], "clearance_mm": o["clearance_mm"],
               "version": self.obs_version, "hsv_set": o["hsv"] is not None,
               "rod_age_s": time.time() - o["rod_t"] if o["rod_t"] else None,
               "feed": f["t"] is not None and time.monotonic() - f["t"] < 2.0, "seen": f["seen"],
               "picking": f["pick"] is not None, "marking": f["mark"] is not None, "error": f["error"],
               "plan_xz": self.plan_xz, "rods": self.rods["list"], "rods_version": self.rods["version"],
               "rods_age_s": time.time() - self.rods["t"] if self.rods["t"] else None,
               "objects": [{k: ob.get(k) for k in ("id", "colour", "shape", "size_mm", "centre", "holes", "outline",
                                                   "pockets")}
                           for ob in self.objects["list"]], "objects_version": self.objects["version"]}
        model = self.model
        if o["rod"] is not None and model is not None and model.has_shape:
            if d is not None:
                out["clear_model_mm"] = float(np.min(rod_clearance(model.backbone([[d["A"], d["B"]]])[0], o["rod"])))
            feed = self.latest_feed()
            if feed and feed["age_s"] < FEED_MAX_AGE_S and feed.get("markers"):
                arc = arc_through_tip(model.cc["base"], [feed["x"], feed["y"], feed["z"]])
                out["clear_camera_mm"] = float(np.min(rod_clearance(arc, o["rod"])))
        return out

    # -- 3D view ------------------------------------------------------------------
    def _set_plan(self, ds):
        self.plan_d = ds
        self.plan_xz = None if ds is None else self.model.predict(ds)[:, [0, 2]].round(1).tolist()
        self.plan_id += 1

    def arm_points(self, D):
        """The arm (attachment point, then backbone()) for bend values D: (len(D), ARC_POINTS + 1, 3)."""
        model = self.model
        B = model.backbone(D)
        base = np.broadcast_to(np.asarray(model.cc["base"], dtype=float), (len(B), 1, 3))
        return np.concatenate([base, B], axis=1)

    def _path_shapes(self, ds):
        """Tip path (N, 3) and a few arm shapes along a path of bend values, for the 3D view."""
        ds = np.asarray(ds, dtype=float)
        out = {"tips": self.model.predict(ds).round(1).tolist(), "arms": []}
        if self.model.has_shape:
            idx = np.unique(np.linspace(0, len(ds) - 1, min(len(ds), PLAN_ARMS)).round().astype(int))
            out["arms"] = self.arm_points(ds[idx]).round(1).tolist()
        p = self.planner()
        if p is not None:
            out["clear_min_mm"] = round(float(np.min(p.clearance(ds))), 1)
        return out

    def plan_shapes(self):
        """The current (or last) move's planned path, if it went around the obstacle."""
        if self.plan_d is None or self.model is None:
            return {"id": self.plan_id, "tips": [], "arms": []}
        return {"id": self.plan_id, **self._path_shapes(self.plan_d)}

    def _known_pose(self):
        if self.ctl.bus and all(self.ctl.state.get(i) for i in motors.IDS) \
                and all(m["zero_rad"] is not None for m in self.ctl.settings["motors"]):
            return self.pose()
        return None

    def plan_preview(self, x, z):
        """Plan a move to tip (x, z) from where the arm is now (or from rest if the motors aren't known) without
        moving: the arm shapes along the way, for the 3D view. Raises ValueError if there is no way."""
        model = self.model
        if model is None:
            raise ValueError(f"No tip model at {self.model_path}; run fit_tip_model.py first")
        d1, pred, dist = model.inverse(x, z)
        cur = self._known_pose()
        d0 = np.array([cur["A"], cur["B"]]) if cur else np.zeros(2)
        note = "" if cur else "from rest (motors not connected). "
        if dist > OUTSIDE_MM:
            note += f"target is {dist:.0f} mm outside the reachable area; nearest reachable point. "
        planner = self.planner()
        if planner is None:
            path = [d1]
        else:
            try:
                way, _, pnote = planner.plan(d0, d1)
            except NoPath as e:
                self.preview = None
                raise ValueError(str(e))
            path, note = split_at_axes(d0, way), note + pnote
        self.preview = {"target": [x, z], "note": note.strip(), **self._path_shapes(densify(d0, path))}
        return self.preview

    def scene_info(self):
        """Static part of the 3D view: the arm's attachment, the tip surface (tip positions over a grid of bend
        values, with the arm's clearance from the rod at each) and the rod. 'version' changes with the model or
        the obstacle."""
        model = self.model
        if model is None:
            return None
        g = np.linspace(-1, 1, SCENE_GRID)
        A, B = np.meshgrid(g, g, indexing="ij")
        D = np.column_stack([A.ravel(), B.ravel()])
        inside = model.bend_rad(D) <= model.max_bend if model.max_bend is not None else np.ones(len(D), bool)
        out = {"version": f"{self._model_mtime}-{self.obs_version}-{self.obs['avoid']}", "has_shape": model.has_shape,
               "base": model.info["base"], "length_mm": model.info["length_mm"], "rest_tip": model.info["rest_tip"],
               "rod": self.obs["rod"], "clearance_mm": self.obs["clearance_mm"], "avoid": self.obs["avoid"],
               "grid": SCENE_GRID, "tips": model.predict(D).round(1).tolist(), "inside": inside.tolist()}
        p = self.planner()
        if p is not None:
            out["clear"] = p.clearance(D).round(1).tolist()
        return out

    # -- pose ---------------------------------------------------------------------
    def max_pull_rad(self, i):
        s = self.ctl.settings["motors"][i]
        return math.radians(s["max_deg"]) - s["zero_rad"]

    def pose(self, angles=None):
        """Bend values from motor angles (absolute, rad); default: the current measured angles."""
        if angles is None:
            angles = {m: motors.to_angle(self.ctl.state[m]["pulse"]) for m in motors.IDS}
        d = {}
        for p, (pos, neg) in PAIRS.items():
            rel = {m: angles[m] - self.ctl.settings["motors"][m]["zero_rad"] for m in (pos, neg)}
            if rel[pos] > math.radians(0.5):
                d[p] = min(1.0, rel[pos] / self.max_pull_rad(pos))
            elif rel[neg] > math.radians(0.5):
                d[p] = -min(1.0, rel[neg] / self.max_pull_rad(neg))
            else:
                d[p] = 0.0
        return d

    def targets_abs(self, d, taut=0.0, relief=None):
        """Absolute motor angles for bend values d. The pulling motor of each pair winds in to its taut zero plus
        |d| * its max pull. Its opposite: with taut = 0 it rests slack at absolute 0 (data collection, tip model);
        with taut = 1 it sits at its own taut zero and unwinds by PAYOUT_RATIO times the angle its opposite winds
        in, down to slack ("taut and pay out"), so at the centre all four cables are just taut. 0 < taut < 1
        blends the two (ramping in). relief: extra unwinding per motor (rad) from the tension guard."""
        out = {}
        for p, (pos, neg) in PAIRS.items():
            v = d[p]
            for m, other, active in ((pos, neg, v > 0), (neg, pos, v < 0)):
                zero = self.ctl.settings["motors"][m]["zero_rad"]
                if active:
                    out[m] = zero + abs(v) * self.max_pull_rad(m)
                    continue
                wound_in = abs(v) * self.max_pull_rad(other) if v != 0 else 0.0
                paid_out = max(REST_ABS_RAD, zero - PAYOUT_RATIO * wound_in - (relief or {}).get(m, 0.0))
                out[m] = REST_ABS_RAD + taut * (paid_out - REST_ABS_RAD)
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
            if m["torque"] is False:  # only a real "off" reading; None = not read this cycle
                raise Fail(f"motor {i} torque is off")
            ma = self.ctl.sustained_raw.get(i, abs(m["current_raw"])) * MA_PER_UNIT  # brief spikes don't count
            # A pulling motor backs off first, even if a fast rise took it past the hard limit before the
            # filter confirmed the soft one: backing off is the quickest way to bring its current down. The hard
            # limit still applies while backing off (soft=False) and to motors that aren't pulling.
            if soft and ma > self.soft_ma and (pulling is None or i in pulling):
                raise ForceLimit(i, ma)
            if ma > self.hard_ma and time.monotonic() >= self._hard_grace_until:
                raise Fail(f"motor {i} current {ma:.0f} mA > {self.hard_ma:.0f} mA")
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
        return self._stream_path([start, end])

    def _stream_path(self, pts):
        """Like _stream, through a list of absolute-angle waypoints in one smooth move (no stops between them)."""
        speed = self.ctl.settings["profile_velocity"] * motors.VELOCITY_UNIT_RPM * 6  # deg/s
        spans = [max(abs(math.degrees(b[i] - a[i])) for i in motors.IDS) for a, b in zip(pts, pts[1:])]
        cum = np.concatenate([[0.0], np.cumsum(spans)])
        duration = max(MIN_SEGMENT_S, 1.5 * cum[-1] / (SPEED_MARGIN * speed))  # smoothstep peaks at 1.5x mean
        pull = [{i for i in motors.IDS if b[i] > a[i] + math.radians(0.5)} for a, b in zip(pts, pts[1:])]
        start = pts[0]
        seen, t0 = [], time.monotonic()  # (time, measured absolute angles)
        try:
            while True:
                u = min(1.0, (time.monotonic() - t0) / duration)
                s = u * u * (3 - 2 * u) * cum[-1]
                n = min(len(spans) - 1, int(np.searchsorted(cum, s, side="right")) - 1)
                k = (s - cum[n]) / spans[n] if spans[n] > 0 else 1.0
                a, b, pulling = pts[n], pts[n + 1], pull[n]
                for i in motors.IDS:
                    self._goal(i, a[i] + (b[i] - a[i]) * k)
                st = self._check(soft=True, pulling=pulling)
                seen.append((time.monotonic(), {i: motors.to_angle(st[i]["pulse"]) for i in motors.IDS}))
                if u >= 1:
                    break
                time.sleep(TICK_S)
            self._wait_settled(pulling=pulling)
            return None
        except ForceLimit as lim:
            now = time.monotonic()
            self._hard_grace_until = now + BACKOFF_GRACE_S
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

    def _go(self, cur, tgt, record=True):
        """Move to bend values tgt; with an obstacle, along a planned path that keeps the arm clear of it (shown in
        the GUI if record). Returns (pose, ForceLimit or None, bend values aimed for, note)."""
        planner = self.planner()
        if planner is None:
            pose, lim = self._move(cur, tgt)
            return pose, lim, tgt, ""
        d0, d1 = np.array([cur["A"], cur["B"]]), np.array([tgt["A"], tgt["B"]])
        way, used, note = planner.plan(d0, d1)
        path = split_at_axes(d0, way)
        if record:
            self._set_plan(densify(d0, path))
        pts = [self._current_goals()] + [self.targets_abs({"A": float(p[0]), "B": float(p[1])}) for p in path]
        lim = self._stream_path(pts)
        used = {"A": float(used[0]), "B": float(used[1])}
        return (self.pose() if lim else used), lim, used, note

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
        if abs(a) > 1 + 1e-9 or abs(b) > 1 + 1e-9:
            raise ValueError("bend values must be within -1..1")
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
        except RuntimeError as e:  # e.g. a motor that tripped its own protection rejects the command
            self._release_all()
            raise RuntimeError(f"{e}; released the other motors to rest")
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
            self._set_plan(None)
            st["message"] = "planning" if self.planner() is not None else "moving"
            cur, lim, aimed, pnote = self._go(self.pose(), {"A": float(d0[0]), "B": float(d0[1])})
            note += pnote
            if abs(aimed["A"] - d0[0]) + abs(aimed["B"] - d0[1]) > 1e-9:  # target blocked by the obstacle
                target = model.predict([[aimed["A"], aimed["B"]]])[0, [0, 2]]
                st["target"] = target.tolist()
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
                step = model.step_toward(d, err)
                n = np.linalg.norm(step)
                if n > MAX_CORRECTION:
                    step *= MAX_CORRECTION / n
                d_new = model.clip(d + step)
                if np.linalg.norm(d_new - d) < 1e-3:
                    st.update(state="done", message=note + "at the edge of the workspace; can't get closer")
                    return
                planner = self.planner()
                if planner is not None:
                    c_now, c_new = planner.clearance(np.array([d, d_new]))
                    if c_new < planner.need and c_new < c_now:
                        st.update(state="done", message=note + "can't get closer without approaching the obstacle")
                        return
                cur, lim, _, _ = self._go({"A": float(d[0]), "B": float(d[1])}, {"A": float(d_new[0]), "B": float(d_new[1])},
                                          record=False)  # the GUI keeps showing the main path
                d = np.array([cur["A"], cur["B"]])
                if lim:
                    st.update(state="limited", message=note + f"force limit ({lim}); stopped short")
                    return
            st.update(state="done", message=note + f"best after {MAX_ITERATIONS} corrections")
        except Stopped:
            st.update(state="stopped", message="stopped")
            self._hold()
        except NoPath as e:  # the arm stays where it is
            st.update(state="failed", message=str(e))
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

    # -- gamepad follow mode ------------------------------------------------------------
    def joy_start(self):
        self.check_ready(need_model=False)
        if not self._motion.acquire(blocking=False):
            raise ValueError("Already moving; press Stop first")
        self._stop.clear()
        self.joy = {"active": True, "enable": False, "set": None, "t": 0.0, "cmd": None, "walls": {},
                    "turn": None, "flick": False, "stick": None, "control": "rate", "tgt_d": None, "tgt_xz": None,
                    "message": "hold the trigger (RT / R2) to move"}

        def run():
            try:
                self._joy_loop()
            finally:
                self.joy["active"] = False
                self._motion.release()
        threading.Thread(target=run, daemon=True).start()

    def joy_input(self, mode, x, y, enable, control="rate"):
        """Latest stick input (x, y in [-1, 1]); mode "bend" (x -> pair A, y -> pair B) or "tip" (top view).
        control "rate" (jog): the stick sets how fast the target moves and letting go leaves it where it is;
        "position": the stick points at a spot (full deflection = edge of the reachable area / full bend)."""
        if not self.joy.get("active"):
            raise ValueError("Gamepad control is not active")
        m = math.hypot(x, y)
        if m > 1:
            x, y, m = x / m, y / m, 1.0
        if mode == "bend" and m > 0:
            # motor pairs: each axis drives its own pair with no combined limit, so stretch the stick's round
            # range onto the square: a full diagonal pulls both pairs fully
            k = m / max(abs(x), abs(y))
            x, y = max(-1.0, min(1.0, x * k)), max(-1.0, min(1.0, y * k))
        if mode == "tip" and self.model is None:
            raise ValueError("Top-view mapping needs a tip model; use the motor-pair mapping")
        if control == "rate":
            self.joy.update(stick=(x, y), mode=mode, control="rate", t=time.monotonic(), enable=bool(enable))
            return
        if mode == "tip":
            model = self.model
            if model is None:
                raise ValueError("Top-view mapping needs a tip model; use the motor-pair mapping")
            ang = math.atan2(y, x)
            target = model.rest_xz + m * model.reach_mm(ang) * np.array([math.cos(ang), math.sin(ang)])
            prev = self.joy.get("set")
            # near the previous target when there is one, so the target can't jump between model solutions
            d = model.inverse_near(*target, [prev["A"], prev["B"]], iters=10, max_step=0.05)[0] if prev \
                else model.inverse(*target)[0]
            want = {"A": float(d[0]), "B": float(d[1])}
        else:
            want = {"A": float(x), "B": float(y)}
        j = self.joy
        if math.hypot(want["A"], want["B"]) < JOY_POLAR_MIN_R:
            j["turn"] = None  # through the centre: direction undefined
        else:
            ang = math.atan2(want["B"], want["A"])
            if j.get("turn") is None:
                # fresh direction (first input, or coming out of the centre): go straight, unless the arm is
                # already bent that way (the loop clears the flick once the directions line up)
                j["turn"], j["flick"] = ang, True
            else:
                jump = (ang - j["turn"] + math.pi) % (2 * math.pi) - math.pi
                if abs(jump) > JOY_FLICK_JUMP:  # far too fast for a sweep: a flick across the centre
                    j["turn"], j["flick"] = ang, True
                else:
                    j["turn"] += jump
        j.update(set=want, stick=(x, y), mode=mode, control="position", t=time.monotonic(), enable=bool(enable))

    @staticmethod
    def _joy_step(d, target, step, arm_turn, stick_turn, straight):
        """One follow step of at most `step` (bend units) from d toward target.

        Normally the bend amount and the bend direction are followed separately: the direction rotates the
        way the stick has been turning (`stick_turn` and `arm_turn` are unwrapped angles), at most half a turn
        behind it, so a stick swept around faster than the motors can follow makes the arm swing around at its
        bend instead of cutting across the middle. `straight` (a flick across, or a small bend) goes in a
        straight line. Returns (next d, next arm_turn)."""
        r0, r1 = math.hypot(d["A"], d["B"]), math.hypot(target["A"], target["B"])
        if straight or stick_turn is None or r0 < JOY_POLAR_MIN_R or r1 < JOY_POLAR_MIN_R:
            dx, dy = target["A"] - d["A"], target["B"] - d["B"]
            n = math.hypot(dx, dy)
            k = 1.0 if n <= step else step / n
            out = {"A": d["A"] + dx * k, "B": d["B"] + dy * k}
            th = math.atan2(out["B"], out["A"])
            ref = stick_turn if stick_turn is not None else th
            arm_turn = ref + (th - ref + math.pi) % (2 * math.pi) - math.pi  # re-sync near the stick's turn
        else:
            lag = max(-math.pi, min(math.pi, stick_turn - arm_turn))  # drop extra turns: trail by <= half a turn
            dr = max(-step, min(step, r1 - r0))
            dth_max = math.sqrt(max(0.0, step * step - dr * dr)) / r0  # arc length r * dth uses the rest of the step
            dth = max(-dth_max, min(dth_max, lag))
            arm_turn += dth
            r = r0 + dr
            out = {"A": r * math.cos(arm_turn), "B": r * math.sin(arm_turn)}
        return clamp_bend(out), arm_turn

    def _jog_target(self, d, dt, rate):
        """Jog: move the target at a speed set by the stick deflection (squared, for fine control near the
        centre), kept at most JOY_LEAD / JOY_LEAD_MM ahead of the arm so it stops when the stick is let go."""
        j = self.joy
        sx, sy = j["stick"]
        m = math.hypot(sx, sy)
        vx, vy = sx * m, sy * m
        if j["mode"] == "tip":
            model = self.model
            here = model.predict([[d["A"], d["B"]]])[0, [0, 2]]
            if j.get("tgt_xz") is None:
                j["tgt_xz"] = here
            txz = j["tgt_xz"] + np.array([vx, vy]) * rate * model.mm_per_bend * dt
            lead = txz - here
            n = float(np.linalg.norm(lead))
            if n > JOY_LEAD_MM:
                txz = here + lead * JOY_LEAD_MM / n
            prev = j.get("tgt_d") or d
            # search near the last target so the dot slides continuously (no jumps between model solutions)
            dd, pred, _ = model.inverse_near(*txz, [prev["A"], prev["B"]])
            j["tgt_xz"] = pred
            return {"A": float(dd[0]), "B": float(dd[1])}
        tgt = j.get("tgt_d") or dict(d)
        tgt = {"A": tgt["A"] + vx * rate * dt, "B": tgt["B"] + vy * rate * dt}
        lead = math.hypot(tgt["A"] - d["A"], tgt["B"] - d["B"])
        if lead > JOY_LEAD:
            tgt = {p: d[p] + (tgt[p] - d[p]) * JOY_LEAD / lead for p in PAIRS}
        return clamp_bend(tgt)

    @staticmethod
    def _avoid_step(planner, d, nxt):
        """Gamepad: keep the arm clear of the rod. A step that would bring it closer than the clearance (and closer
        than it is now) keeps only its part along the obstacle, so the arm slides around it; if even that gets
        closer, the arm stops. Returns (next d, "" / "sliding" / "blocked")."""
        a, b = np.array([d["A"], d["B"]]), np.array([nxt["A"], nxt["B"]])
        c_now, c_next = planner.clearance(np.array([a, b]))
        if c_next >= planner.need or c_next >= c_now:
            return nxt, ""
        keep = min(planner.need, c_now)
        h = 1e-3
        grad = np.array([(planner.clearance(a + e)[0] - planner.clearance(a - e)[0]) / (2 * h) for e in np.eye(2) * h])
        gn2 = grad @ grad
        if gn2 > 1e-9:
            step = b - a
            slide = step - min(0.0, step @ grad) / gn2 * grad
            c = planner.clearance(a + slide)[0]
            slide += max(0.0, keep - c) / gn2 * grad  # back onto the clearance level the tangent step left
            nd = np.clip(a + slide, -1.0, 1.0)
            if np.linalg.norm(nd - a) > 1e-4 and planner.clearance(nd)[0] >= keep - 0.3:
                return {"A": float(nd[0]), "B": float(nd[1])}, "sliding"
        return dict(d), "blocked"

    def _joy_loop(self):
        """Follow the stick: bend values move toward the stick target at the speed limit, a pair crossing the
        centre releases its cable before the opposite one pulls, and a force limit becomes a wall."""
        j = self.joy
        try:
            self._prepare()
            d = self.pose()
            taut, relief = 0.0, {}  # motor-pair mode: taut-and-pay-out blend (ramped in) and tension relief
            goals = self.targets_abs(d)
            seen = collections.deque(maxlen=40)  # (time, measured absolute angles)
            arm_turn = math.atan2(d["B"], d["A"])  # unwrapped bend direction of the arm
            last = time.monotonic()
            while True:
                now = time.monotonic()
                dt, last = min(0.1, now - last), now
                speed = self.ctl.settings["profile_velocity"] * motors.VELOCITY_UNIT_RPM * 6
                max_pull = max(math.degrees(self.max_pull_rad(i)) for i in motors.IDS)
                rate = SPEED_MARGIN * speed / max_pull  # bend units per second the arm can follow
                jog = j.get("control") == "rate"
                square = j.get("mode") == "bend"  # motor pairs: straight-line following, cables kept taut
                live = j["enable"] and j.get("stick") is not None and now - j["t"] < JOY_STALE_S \
                    and (jog or j.get("set") is not None)
                if jog:
                    target = self._jog_target(d, dt, rate) if live else dict(d)
                    if not live:
                        j["tgt_xz"] = None
                else:
                    target = dict(j["set"]) if live else dict(d)
                walls = j["walls"]
                clamped = False
                for p in PAIRS:
                    sign = 1 if target[p] > 0 else -1 if target[p] < 0 else 0
                    wall = walls.get((p, sign))
                    if wall is not None:
                        if abs(target[p]) < wall["limit"] - JOY_WALL_CLEAR or now - wall["t"] > JOY_WALL_RETRY_S:
                            del walls[(p, sign)]
                        elif abs(target[p]) > wall["limit"]:
                            target[p] = sign * wall["limit"]
                            clamped = True
                for key in [k for k, w in walls.items() if now - w["t"] > JOY_WALL_RETRY_S]:
                    del walls[key]
                if clamped and jog and j.get("mode") == "tip" and self.model is not None:
                    # keep the jog target at the wall too, so easing off takes effect straight away
                    j["tgt_xz"] = self.model.predict([[target["A"], target["B"]]])[0, [0, 2]]
                j["tgt_d"] = dict(target)
                step = rate * JOY_TICK_S
                stick_turn, straight = None, True
                if live and not jog and not square and j.get("turn") is not None:  # motor pairs: straight lines
                    a = math.atan2(target["B"], target["A"])  # after walls
                    stick_turn = j["turn"] + (a - j["turn"] + math.pi) % (2 * math.pi) - math.pi
                    straight = j.get("flick", False)
                    if straight and abs(stick_turn - arm_turn) < math.radians(30):
                        j["flick"] = straight = False  # flick finished: back to following the direction
                nxt, arm_turn = self._joy_step(d, target, step, arm_turn, stick_turn, straight)
                if not square and self.model is not None:  # tip mode: stay inside the model's workspace
                    c = self.model.clip([nxt["A"], nxt["B"]])
                    nxt = {"A": float(c[0]), "B": float(c[1])}
                planner = self.planner()
                avoided = ""
                if planner is not None:
                    nxt, avoided = self._avoid_step(planner, d, nxt)
                    if avoided == "blocked" and jog and j.get("mode") == "tip":
                        j["tgt_xz"] = None  # restart the jog target at the arm, so pushing elsewhere responds at once
                for p, (pos, neg) in PAIRS.items():
                    if d[p] * nxt[p] < 0:
                        nxt[p] = 0.0  # release this cable fully before the opposite one pulls
                    elif d[p] == 0 and nxt[p] != 0:  # taut mode: only a short wait, it just has to reach taut
                        releasing = neg if nxt[p] > 0 else pos
                        rel = motors.to_angle(self.ctl.state[releasing]["pulse"]) \
                            - self.ctl.settings["motors"][releasing]["zero_rad"]
                        if rel > math.radians(0.5):
                            nxt[p] = 0.0  # opposite cable not released yet
                    if abs(nxt[p]) < 1e-9:
                        nxt[p] = 0.0
                d = nxt
                if live:  # cables are only tightened while the trigger / on-screen stick is held
                    want_taut = 1.0 if j.get("mode") == "bend" else 0.0
                    taut += max(-dt / TAUT_RAMP_S, min(dt / TAUT_RAMP_S, want_taut - taut))
                prev, goals = goals, self.targets_abs(d, taut, relief)
                for i, g in goals.items():
                    self._goal(i, g)
                pulling = {i for i in motors.IDS if goals[i] > prev[i] + 1e-9}
                try:
                    st = self._check(soft=True, pulling=pulling)
                except ForceLimit as lim:
                    self._hard_grace_until = time.monotonic() + BACKOFF_GRACE_S
                    back = next((a for t, a in reversed(seen) if now - t >= JOY_BACKOFF_S), None)
                    if back is not None:
                        for i, g in back.items():
                            self._goal(i, g)
                        d, goals = self.pose(back), dict(back)
                    p = next(p for p, pair in PAIRS.items() if lim.motor in pair)
                    walls[(p, 1 if lim.motor == PAIRS[p][0] else -1)] = {"limit": abs(d[p]), "t": now, "motor": lim.motor}
                    if jog:
                        j["tgt_d"], j["tgt_xz"] = dict(d), None  # restart the jog target from the backed-off pose
                    time.sleep(JOY_TICK_S)
                    continue
                if taut > 0:  # tension guard: a paying-out (non-pulling) cable that goes tight gets more slack
                    for p, (pos, neg) in PAIRS.items():
                        for m in ((neg,) if d[p] > 0 else (pos,) if d[p] < 0 else (pos, neg)):
                            if abs(st[m]["current_raw"]) * MA_PER_UNIT > PAYOUT_GUARD_MA:
                                avail = self.ctl.settings["motors"][m]["zero_rad"] - REST_ABS_RAD
                                relief[m] = min(avail, relief.get(m, 0.0) + PAYOUT_RELIEF_STEP)
                    j["relief_deg"] = {m: round(math.degrees(r), 1) for m, r in relief.items() if r > 0}
                j["taut"] = taut
                seen.append((now, {i: motors.to_angle(st[i]["pulse"]) for i in motors.IDS}))
                j["cmd"] = dict(d)
                fresh = j.get("stick") is not None and now - j["t"] < JOY_STALE_S
                j["message"] = ("moving" if live else "holding (trigger released)" if fresh
                                else "holding (no stick input)") + (" · cables taut" if taut >= 1 else "") \
                    + {"sliding": " · sliding around the obstacle", "blocked": " · stopped at the obstacle"}.get(avoided, "")
                if walls:
                    w = max(walls.values(), key=lambda w: w["t"])
                    j["message"] = (f"force limit on motor {w['motor']}: ease off, or keep pushing to retry in "
                                    f"{max(0.0, JOY_WALL_RETRY_S - (now - w['t'])):.1f} s")
                time.sleep(JOY_TICK_S)
        except Stopped:
            j["message"] = "stopped"
            self._hold()
        except Fail as e:
            j["message"] = f"{e}; released motors to rest"
            self._release_all()
        except Exception as e:  # e.g. lost connection
            j["message"] = str(e)

    def snapshot(self):
        model = self.model
        out = {"model": model is not None, "status": self.status, "busy": self.busy}
        j = self.joy
        out["joy"] = {"active": j.get("active", False), "message": j.get("message", ""),
                      "enable": j.get("enable", False), "cmd": j.get("cmd"),
                      "walls": [w["motor"] for w in j.get("walls", {}).values()]}
        tgt = j.get("tgt_d") if j.get("control") == "rate" else j.get("set")
        if model is not None and j.get("active") and tgt:
            out["joy"]["target_xz"] = model.predict([[tgt["A"], tgt["B"]]])[0, [0, 2]].tolist()
        feed = self.latest_feed()
        out["feed"] = feed if feed and feed["age_s"] <= 2.0 else None
        d = self._known_pose() if model is not None else None
        if d is not None:
            out["pose"] = d
            out["predicted"] = model.predict([[d["A"], d["B"]]])[0].tolist()
        if model is not None and model.has_shape:  # the arm's shape, for the 3D view
            if d is not None:
                out["arm_model"] = self.arm_points([[d["A"], d["B"]]])[0].round(1).tolist()
            if out["feed"] and out["feed"]["age_s"] < FEED_MAX_AGE_S and out["feed"].get("markers"):
                f = out["feed"]
                arc = arc_through_tip(model.cc["base"], [f["x"], f["y"], f["z"]])
                out["arm_camera"] = np.vstack([model.cc["base"], arc]).round(1).tolist()
        out["plan_id"] = self.plan_id
        out["obstacle"] = self._obstacle_snapshot(d)
        return out

    def model_info(self):
        model = self.model
        if model is None:
            return None
        i = model.info
        return {"base": i["base"], "length_mm": i["length_mm"], "rest_tip": i["rest_tip"], "samples": i["samples"],
                "cv_error_mm": i["cv_error_mm"], "n": i["n"], "fitted": i["fitted"],
                "outline": model.outline_xz.round(1).tolist(), "motor_dirs_deg": model.motor_dirs_deg.round(1).tolist()}
