"""Find a brightly coloured rod (an obstacle) in the camera image and fit it as a 3D segment in the calibrated
EMIO frame (mm), using the depth camera. Used by tip_feed.py, which sends the rod to the motor GUI so tip moves
can plan around it.

The rod's colour is an HSV range (OpenCV: H 0..179): a preset, or picked by clicking the rod in the GUI's camera
view. Depth points on the rod's visible surface are fitted with a RANSAC line; the radius comes from the rod's
width in the image. An end that runs out of the image is extended (the rod continues out of view, e.g. into the
pillar it is attached to). RodTracker merges detections over time: the rod is static, so its visible extent only
grows, and the arm passing in front of it can't make it shorter.
"""
import math

import cv2
import numpy as np

PRESETS = {
    "red": [[[0, 120, 70], [8, 255, 255]], [[170, 120, 70], [179, 255, 255]]],
    "orange": [[[8, 120, 90], [22, 255, 255]]],
    "yellow": [[[22, 100, 100], [34, 255, 255]]],
    "pink": [[[145, 80, 90], [170, 255, 255]]],
    "purple": [[[125, 60, 50], [145, 255, 255]]],
}
PICK_HUE = 12           # picked colour: hue +- this
MIN_PICK_SAT = 70       # a pick less saturated than this is not a bright colour (grey, white, ...)
MIN_BLOB_PX = 40        # colour blobs smaller than this are ignored
MIN_PIXELS = 150        # fewer rod pixels than this = not seen
MIN_LENGTH_MM = 25.0
INLIER_MM = 6.0         # RANSAC: depth points this close to the line belong to the rod
DEPTH_RANGE_MM = (40, 1000)
BORDER_PX = 8           # an end this close to the image border runs out of view ...
EXTEND_MM = 150.0       # ... and is extended this far
SURFACE_SHIFT = 0.7     # depth points lie on the visible surface: the axis is this many radii further away


def hsv_range_at(hsv, x, y, r=4):
    """HSV range(s) for the colour around pixel (x, y). Raises ValueError for an unsaturated (grey/white) spot."""
    patch = hsv[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1].reshape(-1, 3).astype(float)
    if not len(patch):
        raise ValueError("pick is outside the image")
    ang = patch[:, 0] * math.pi / 90  # hue is circular (red wraps around 0/180)
    h = (math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean())) / 2) % 180
    s, v = np.median(patch[:, 1]), np.median(patch[:, 2])
    if s < MIN_PICK_SAT:
        raise ValueError(f"that spot isn't a bright colour (saturation {s:.0f}/255); click the coloured rod")
    lo_s, lo_v = int(max(50, s * 0.5)), int(max(40, v * 0.4))
    lo, hi = int(round(h - PICK_HUE)), int(round(h + PICK_HUE))
    if lo < 0:
        return [[[0, lo_s, lo_v], [hi, 255, 255]], [[180 + lo, lo_s, lo_v], [179, 255, 255]]]
    if hi > 179:
        return [[[lo, lo_s, lo_v], [179, 255, 255]], [[0, lo_s, lo_v], [hi - 180, 255, 255]]]
    return [[[lo, lo_s, lo_v], [hi, 255, 255]]]


def color_mask(hsv, ranges):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


class CameraGeometry:
    """Pixels + depth <-> calibrated EMIO frame, as emioapi computes it (pinhole, then R, t from the ArUco
    calibration)."""

    def __init__(self, fx, fy, ppx, ppy, R, t, size):
        self.fx, self.fy, self.ppx, self.ppy = fx, fy, ppx, ppy
        self.R, self.t = np.asarray(R, float), np.asarray(t, float).ravel()
        self.Rinv = np.linalg.inv(self.R)
        self.w, self.h = size

    @classmethod
    def from_emio(cls, cam):
        pe = cam._camera.position_estimator
        if cam.configuration != "extended":
            raise ValueError("rod detection supports the extended configuration only")
        i = pe.intr
        return cls(i.fx, i.fy, i.ppx, i.ppy, pe.R, pe.t, (i.width, i.height))

    @property
    def centre(self):
        return self.t

    def to_world(self, u, v, z):
        P = np.stack([(u - self.ppx) / self.fx * z, (v - self.ppy) / self.fy * z, z], axis=-1)
        return P @ self.R.T + self.t

    def to_image(self, W):
        P = (np.atleast_2d(W) - self.t) @ self.Rinv.T
        return np.stack([self.fx * P[:, 0] / P[:, 2] + self.ppx, self.fy * P[:, 1] / P[:, 2] + self.ppy], axis=-1)


def detect(hsv, depth, ranges, geom, rng=None):
    """Find the rod. Returns (rod or None, mask). rod: p0, p1 (visible ends, mm, EMIO frame), radius (mm), open
    (per end: runs out of the image), pixels, img (visible ends in the image)."""
    mask = color_mask(hsv, ranges)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    keep = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= MIN_BLOB_PX]
    if sum(stats[i, cv2.CC_STAT_AREA] for i in keep) < MIN_PIXELS:
        return None, mask
    sel = np.isin(lab, keep) & (depth >= DEPTH_RANGE_MM[0]) & (depth <= DEPTH_RANGE_MM[1])
    vs, us = np.nonzero(sel)
    if len(us) < MIN_PIXELS // 2:
        return None, mask
    rng = rng or np.random.default_rng(0)
    n_px = len(us)
    k = min(n_px, 3000)
    pick = rng.choice(len(us), k, replace=False)
    us, vs = us[pick], vs[pick]
    W = geom.to_world(us.astype(float), vs.astype(float), depth[vs, us].astype(float))
    best = None
    for _ in range(150):  # RANSAC: the line through most depth points (ignores other blobs of the same colour)
        i, j = rng.choice(len(W), 2, replace=False)
        d = W[j] - W[i]
        nd = np.linalg.norm(d)
        if nd < 10:
            continue
        d /= nd
        r = W - W[i]
        inl = np.linalg.norm(r - np.outer(r @ d, d), axis=1) < INLIER_MM
        if best is None or inl.sum() > best.sum():
            best = inl
    area_px = 0 if best is None else best.sum() * n_px / k  # rod pixels (with depth) along the line
    if area_px < MIN_PIXELS // 2:
        return None, mask
    P = W[best]
    m = P.mean(axis=0)
    u = np.linalg.svd(P - m)[2][0]
    s = (P - m) @ u
    s0, s1 = np.percentile(s, [1, 99])
    if s1 - s0 < MIN_LENGTH_MM:
        return None, mask
    # radius from the rod's width in the image: pixel area / pixel length, at the rod's depth
    ends = np.array([m + s0 * u, m + s1 * u])
    img = geom.to_image(ends)
    seg = img[1] - img[0]
    len_px = max(1.0, float(np.linalg.norm(seg)))
    my, mx = np.nonzero(np.isin(lab, keep))  # all rod-coloured pixels near the line (depth often has holes)
    rel = np.column_stack([mx, my]) - img[0]
    t = np.clip(rel @ seg / len_px ** 2, 0, 1)
    area_px = np.count_nonzero(np.linalg.norm(rel - np.outer(t, seg), axis=1) < 60)
    z = float(np.median(depth[vs[best], us[best]]))
    radius = float(np.clip(area_px / len_px / 2 * z / geom.fx, 2.0, 25.0))
    # the depth points are on the side facing the camera: move the axis away from the camera
    view = m - geom.centre
    away = view - (view @ u) * u
    ends = ends + SURFACE_SHIFT * radius * away / np.linalg.norm(away)
    opened = [bool(p[0] < BORDER_PX or p[1] < BORDER_PX or p[0] > geom.w - 1 - BORDER_PX or p[1] > geom.h - 1 - BORDER_PX)
              for p in img]
    return {"p0": ends[0], "p1": ends[1], "radius": radius, "open": opened, "pixels": int(area_px),
            "img": img}, mask


def _unit(v):
    return v / np.linalg.norm(v)


class RodTracker:
    """Merges detections of a static rod: the axis is smoothed and the visible extent only grows (so the arm in
    front of the rod can't hide part of it). A rod that is moved (different line for several detections in a row)
    replaces the old one."""

    SMOOTH = 0.15
    SAME_DEG, SAME_MM = 8.0, 10.0
    REPLACE_AFTER = 8

    def __init__(self):
        self.reset()

    def reset(self):
        self.c = self.u = None
        self.other = 0

    def update(self, det):
        if det is None:
            return
        p0, p1 = np.asarray(det["p0"], float), np.asarray(det["p1"], float)
        ud = _unit(p1 - p0)
        if self.c is not None:
            if ud @ self.u < 0:  # same orientation as the tracked rod
                p0, p1, ud = p1, p0, -ud
                det = {**det, "open": det["open"][::-1]}
            md = (p0 + p1) / 2
            off = (self.c - md) - ((self.c - md) @ ud) * ud
            same = math.degrees(math.acos(min(1.0, abs(ud @ self.u)))) < self.SAME_DEG and np.linalg.norm(off) < self.SAME_MM
            if not same:
                self.other += 1
                if self.other < self.REPLACE_AFTER:
                    return
            else:
                self.other = 0
                k = self.SMOOTH
                self.u = _unit((1 - k) * self.u + k * ud)
                self.c = self.c - k * off  # pull the axis toward the new line
                self.r = (1 - k) * self.r + k * det["radius"]
                s = sorted([(p0 - self.c) @ self.u, (p1 - self.c) @ self.u])
                self.s = [min(self.s[0], s[0]), max(self.s[1], s[1])]
                self.open = [self.open[0] or det["open"][0], self.open[1] or det["open"][1]]
                return
        self.u, self.c = ud, (p0 + p1) / 2
        self.s = [(p0 - self.c) @ ud, (p1 - self.c) @ ud]
        self.r, self.open, self.other = det["radius"], list(det["open"]), 0

    def rod(self):
        """The merged rod (open ends extended), or None."""
        if self.c is None:
            return None
        s0 = self.s[0] - (EXTEND_MM if self.open[0] else 0.0)
        s1 = self.s[1] + (EXTEND_MM if self.open[1] else 0.0)
        return {"p0": (self.c + s0 * self.u).round(1).tolist(), "p1": (self.c + s1 * self.u).round(1).tolist(),
                "radius": round(float(self.r), 1), "open": self.open,
                "visible_mm": round(float(self.s[1] - self.s[0]), 1)}


def draw(frame, mask, det, rod, geom):
    """Live-view overlay: the colour mask outline, the fitted rod and its merged extent."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (255, 0, 255), 1)
    if rod is not None:
        ends = geom.to_image(np.array([rod["p0"], rod["p1"]]))
        if np.all(np.isfinite(ends)):
            a, b = (tuple(int(round(c)) for c in e) for e in ends)
            cv2.line(frame, a, b, (0, 0, 255), 2, cv2.LINE_AA)
    if det is not None:
        a, b = (tuple(int(round(c)) for c in e) for e in det["img"])
        cv2.line(frame, a, b, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"rod r={det['radius']:.0f} mm", (a[0] + 5, a[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 0, 255), 1, cv2.LINE_AA)
