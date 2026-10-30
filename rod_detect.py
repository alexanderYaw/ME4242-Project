"""Find brightly coloured rods / bars and other objects (obstacles) in the camera image and measure them in the
calibrated EMIO frame (mm) with the depth camera: each rod as a 3D segment, each other object (a tray, a button, ...)
as its visible surface (a mesh), outline and shape. Used by tip_feed.py, which sends them to the motor GUI for display.

Each colour is an HSV range (OpenCV: H 0..179): presets (orange and red by default), or one colour picked by
clicking a rod in the GUI's camera view. Per colour, regions too thick to be a rod (e.g. a box) are dropped, then
rods are found one after the other by RANSAC on the depth points of the rod-coloured pixels (each rod's points are
removed before looking for the next). A rod is thin and glossy: its depth is noisy, often missing, and often that of
what is behind it. So its line is fitted in the image and followed along its colour, and its depth along that line
is a robust line fit to the depth of its middle pixels that are in front of the background beside it. The radius
comes from the rod's width in the image. An end that runs out
of the image is extended (the rod continues out of view, e.g. into the pillar it is attached to), if enough of the
rod is seen to trust its direction. RodTracker merges
detections over time: the rods are static, so a rod's visible extent only grows, and the arm passing in front of it
can't make it shorter.

A wide colour blob that the rods found in it don't mostly cover is an object instead (its rods are dropped); touching
objects are split at the neck between them. Its surface is a mesh over a grid of cells with depth, its outline is the
blob's contour at the depth of the nearest cells, and its shape comes from where the contour's convex hull turns
(corners: a polygon, evenly: a circle / ellipse). Pockets in an object (e.g. the shaped holes of a shape-sorter
tray) are the parts of its filled outline below its top face, from a plane fitted to the top. Green is looked for as
objects only (a tray), not rods. ObjectTracker reports objects seen in several detections in a row.
"""
import collections
import math

import cv2
import numpy as np

PRESETS = {
    "red": [[[0, 120, 45], [7, 255, 255]], [[170, 120, 45], [179, 255, 255]]],  # dark: red bars in shade are V ~60
    "orange": [[[8, 120, 90], [22, 255, 255]]],
    "yellow": [[[22, 100, 100], [34, 255, 255]]],
    "pink": [[[145, 80, 90], [170, 255, 255]]],
    "purple": [[[125, 60, 50], [145, 255, 255]]],
    "green": [[[38, 40, 35], [80, 255, 255]]],  # a dull, dark green tray: S ~75, V ~70
}
DEFAULT_COLOURS = ("orange", "red", "green")
OBJECTS_ONLY = ("green",)  # colours looked for as objects only, not rods
PICK_HUE = 12           # picked colour: hue +- this
MIN_PICK_SAT = 70       # a pick less saturated than this is not a bright colour (grey, white, ...)
MIN_BLOB_PX = 40        # colour blobs smaller than this are ignored
MIN_PIXELS = 150        # fewer rod pixels than this = not seen
MIN_LENGTH_MM = 25.0
MAX_ROD_MM = 20.0       # colour regions thicker than this (a box, a panel, a shape) are not rods
MAX_RODS = 6            # per colour
ROD_ASPECT = 3.0        # a rod is at least this many diameters long
FLANK_MM = 12.0         # the strip this wide beside a rod ...
MAX_FLANK = 0.3         # ... is at most this full of the rod's colour (along most of the rod)
GAP_MM = 40.0           # points on one line further apart than this belong to separate rods
SAMPLE = 2500           # depth points per colour used for the line fits
RANSAC_ITERS = 120
INLIER_MM = 6.0         # RANSAC: depth points this close to the line belong to the rod
DEPTH_RANGE_MM = (40, 1000)
MIN_OBJECT_PX = 300     # colour blobs at least this big that aren't rods are objects ...
MIN_OBJECT_MM = 10.0    # ... and at least this wide (thinner: a piece of a rod) ...
WIDE_MM = 14.0          # ... and this wide somewhere, holes filled (else it's rods, e.g. a frame of bars)
ROD_SHARE = 0.6         # a blob is rods if the rods found in it cover at least this share of it
OBJ_CELLS = 1200        # an object's surface mesh has about this many grid cells at most
JOIN_MM = 8.0           # mesh cells further apart in depth than this (+3 % of the depth) aren't joined
SPLIT_CORE = 0.6        # touching objects: each has a core at least this share as thick as the thickest
POCKET_MM = 3.0         # an object's pixels this far below its top face are in a pocket ...
POCKET_SPAN_MM = 60.0   # ... narrower than this (the top face is found over this span)
MIN_POCKET_MM = 12.0    # ... at least this wide
MAX_POCKETS = 8
BORDER_PX = 8           # an end this close to the image border runs out of view ...
EXTEND_MM = 150.0       # ... and is extended this far (at most as far as the rod is seen) ...
EXTEND_MIN_MM = 60.0    # ... if at least this much is seen (a shorter piece's direction isn't reliable enough)
SURFACE_SHIFT = 0.7     # depth points lie on the visible surface: the axis is this many radii further away
MAX_VIEW_DEG = 25.0     # a "rod" within this angle of the view direction is depth noise at an edge
FOLLOW_GAP_MM = 8.0     # where its depth is missing (glossy, too close), a rod is followed along its colour in
                        # the image, over gaps up to this long
FRONT_MM = 20.0         # a rod's depth is this much in front of the background beside it (else it's the background's)
MIN_DEPTH_PX = 25       # a rod's depth line needs at least this many pixels with depth
DEPTH_ITERS = 60


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


def drop_thick(mask, depth, fx):
    """The mask without the blobs that are thicker than MAX_ROD_MM somewhere (at their depth): a box or panel of
    the rod's colour, including its thin edges."""
    half = cv2.distanceTransform(mask, cv2.DIST_L2, 3)  # half the blob's thickness at its centre, px
    z = depth.astype(np.float32)
    ok = (mask > 0) & (z >= DEPTH_RANGE_MM[0]) & (z <= DEPTH_RANGE_MM[1])
    if not ok.any():
        return mask
    z[~ok] = np.median(z[ok])  # depth holes
    thick = half * z / fx > MAX_ROD_MM / 2
    if not thick.any():
        return mask
    _, lab = cv2.connectedComponents(mask)
    out = mask.copy()
    out[np.isin(lab, np.unique(lab[thick]))] = 0
    return out


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


def detect(hsv, depth, colours, geom, rng=None):
    """Find the rods and objects. colours: {name: HSV ranges}. Returns (rods, objects, masks).
    Each rod: p0, p1 (visible ends, mm, EMIO frame), radius (mm), open (per end: runs out of the image), pixels, img
    (visible ends in the image), colour. Each object: see _object(), plus colour. masks: {name: rod-coloured pixels}."""
    rng = rng or np.random.default_rng(0)
    rods, objects, masks = [], [], {}
    for name, ranges in colours.items():
        full = color_mask(hsv, ranges)
        masks[name] = full
        found = [] if name in OBJECTS_ONLY else _find_rods(drop_thick(full, depth, geom.fx), depth, geom, rng)
        # blobs the rods don't mostly cover are objects (a tray, a button, ...), and the rods in them aren't rods
        covered = np.zeros_like(full)
        for r in found:
            cv2.line(covered, *(tuple(int(round(c)) for c in e) for e in r["img"]), 255,
                     max(1, int(round(3 * r["half_px"] + 4))))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(full)
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < MIN_OBJECT_PX:
                continue
            blob = lab[y:y + h, x:x + w] == i
            filled, outline = _filled(blob)
            zb = depth[y:y + h, x:x + w][blob]
            zb = zb[(zb >= DEPTH_RANGE_MM[0]) & (zb <= DEPTH_RANGE_MM[1])]
            half = cv2.distanceTransform(np.pad(filled, 1).astype(np.uint8), cv2.DIST_L2, 3).max()
            if not len(zb) or 2 * half * np.median(zb) / geom.fx < WIDE_MM:
                continue  # thin everywhere: rods (a frame of bars), not an object
            mine = [r for r in found if _inside(r["img"], filled, x, y)]
            if np.count_nonzero(covered[y:y + h, x:x + w][blob]) >= ROD_SHARE * area and \
                    not any(_side_by_side(a, b) for k, a in enumerate(mine) for b in mine[k + 1:]):
                continue
            found = [r for r in found if not any(r is m for m in mine)]
            for part in _parts(filled):  # objects side by side
                pb = blob & part
                if np.count_nonzero(pb) < MIN_OBJECT_PX:
                    continue
                pf, po = _filled(pb)
                obj = _object(pb, pf, po, x, y, depth, geom)
                if obj is not None and obj["size_mm"][1] >= MIN_OBJECT_MM:
                    objects.append({**obj, "colour": name})
        rods += [{**r, "colour": name} for r in found]
    return rods, objects, masks


def _filled(blob):
    """The blob with its holes filled, and its outer contour (image pixels relative to the blob's box)."""
    contours, _ = cv2.findContours(blob.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    outline = max(contours, key=cv2.contourArea)
    filled = np.zeros(blob.shape, np.uint8)
    cv2.drawContours(filled, contours, -1, 1, -1)
    return filled.astype(bool), outline.reshape(-1, 2)


def _parts(filled):
    """Split a filled blob into the objects it is made of when they touch: each object has a core where the blob is
    thick (its centre), joined to the next by a thinner neck. Tries several thickness levels and keeps the one with
    the most cores; each pixel goes to the nearest core. Returns boolean masks (one: not split)."""
    dt = cv2.distanceTransform(np.pad(filled, 1).astype(np.uint8), cv2.DIST_L2, 5)[1:-1, 1:-1]
    best = []
    for f in (0.5, 0.6, 0.7, 0.8):
        n, cores = cv2.connectedComponents((dt >= f * dt.max()).astype(np.uint8))
        keep = [k for k in range(1, n) if dt[cores == k].max() >= SPLIT_CORE * dt.max()]
        if len(keep) > len(best):
            best, best_cores = keep, cores
    if len(best) <= 1:
        return [filled]
    # touching objects pinch the outline in from both sides; a crease (a shadow) in one object doesn't
    c = max(cv2.findContours(filled.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0], key=cv2.contourArea)
    defects = cv2.convexityDefects(c, cv2.convexHull(c, returnPoints=False))
    if defects is None or np.count_nonzero(defects[:, 0, 3] / 256 > 0.3 * dt.max()) < 2:
        return [filled]
    seeds = np.isin(best_cores, best)
    _, near = cv2.distanceTransformWithLabels((~seeds).astype(np.uint8), cv2.DIST_L2, 5,
                                               labelType=cv2.DIST_LABEL_CCOMP)
    return [filled & (near == k) for k in np.unique(near[seeds])]


def _side_by_side(a, b):
    """Whether rods a and b are parallel and touching: strips of one wide object (a tray), not two rods."""
    ua, ub = _unit(b["p1"] - b["p0"]), _unit(a["p1"] - a["p0"])
    r = (a["p0"] + a["p1"]) / 2 - b["p0"]
    return abs(ua @ ub) > math.cos(math.radians(15)) and \
        np.linalg.norm(r - (r @ ua) * ua) < a["radius"] + b["radius"] + FLANK_MM


def _inside(img, filled, x0, y0):
    """Whether most of the segment img (two image points) lies on the filled blob at (x0, y0)."""
    pts = np.round(img[0] + np.linspace(0, 1, 9)[:, None] * (img[1] - img[0])).astype(int) - [x0, y0]
    ok = (pts[:, 0] >= 0) & (pts[:, 0] < filled.shape[1]) & (pts[:, 1] >= 0) & (pts[:, 1] < filled.shape[0])
    return np.count_nonzero(filled[pts[ok, 1], pts[ok, 0]]) > len(pts) // 2


def _shape(outline):
    """Shape name from the outer contour's convex hull. Walking round it, a polygon turns at its corners only (also
    when they're rounded) and a circle / ellipse turns evenly: corners are the peaks of the turning over a short
    stretch, a polygon if they hold most of the turning."""
    hull = cv2.convexHull(outline.reshape(-1, 1, 2).astype(np.float32)).reshape(-1, 2)
    if len(hull) < 3:
        return "object"
    # 72 points evenly spaced along the hull, and the turning at each (degrees)
    closed = np.vstack([hull, hull[:1]])
    cum = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    t = np.linspace(0, cum[-1], 72, endpoint=False)
    pts = np.column_stack([np.interp(t, cum, closed[:, 0]), np.interp(t, cum, closed[:, 1])])
    seg = np.roll(pts, -1, axis=0) - pts
    ang = np.degrees(np.arctan2(seg[:, 1], seg[:, 0]))
    turn = np.abs((np.diff(np.concatenate([ang, ang[:1]])) + 180) % 360 - 180)
    win = np.convolve(np.concatenate([turn[-2:], turn, turn[:2]]), np.ones(5), "valid")  # over 5 points (7 %)
    # corners: peaks of the turning (the first point of a flat top), round the closed hull
    corners = [k for k in range(72) if win[k] >= 30 and win[k] > win[k - 1] and
               win[k] >= max(win[(k + j) % 72] for j in range(1, 4)) and win[k] >= max(win[k - j] for j in range(2, 4))]
    in_corners = sum(win[k] for k in corners) / 360
    if 3 <= len(corners) <= 6 and in_corners > 0.7:
        return {3: "triangle", 4: "quadrilateral", 5: "pentagon", 6: "hexagon"}[len(corners)]
    if len(hull) < 5:
        return "object"
    (_, _), (a, b), _ = cv2.fitEllipse(hull)
    if cv2.contourArea(hull) > 0.9 * math.pi * a * b / 4:  # fills its ellipse
        return "circle" if min(a, b) > 0.8 * max(a, b) else "ellipse"
    return "object"


def _object(blob, filled, outline, x0, y0, depth, geom):
    """An object from its blob (pixels in the box at x0, y0): {verts (n x 3, mm), faces (triangles, vertex
    indices), outline (3D contour, mm), centre, size_mm (longest, shortest extent of the surface), shape, holes
    (share of the outline's area that isn't the colour, e.g. a mesh tray), pockets (see _pockets), pixels,
    img_outline (image contour)}, or None without enough depth."""
    h, w = blob.shape
    step = max(3, int(math.ceil(math.sqrt(np.count_nonzero(filled) / OBJ_CELLS))))
    H, Wc = h // step, w // step
    if H < 2 or Wc < 2:
        return None
    cells = lambda a: a[:H * step, :Wc * step].reshape(H, step, Wc, step).transpose(0, 2, 1, 3).reshape(H, Wc, -1)
    m = cells(blob)
    d = cells(depth[y0:y0 + h, x0:x0 + w].astype(np.float32))
    ok = m & (d >= DEPTH_RANGE_MM[0]) & (d <= DEPTH_RANGE_MM[1])
    valid = ok.sum(axis=2) >= 0.3 * step * step
    if np.count_nonzero(valid) < 6:
        return None
    # each cell's depth: the median of its valid depths (sorting puts the invalid ones, inf, last)
    k = ok.sum(axis=2)
    z = np.take_along_axis(np.sort(np.where(ok, d, np.inf), axis=2), np.maximum(k - 1, 0)[..., None] // 2, axis=2)[..., 0]
    # cells far from the object's depth are what's behind it at its edges (or noise)
    zm = np.median(z[valid])
    valid &= np.abs(z - zm) <= max(40.0, 7.4 * np.median(np.abs(z[valid] - zm)))
    if np.count_nonzero(valid) < 6:
        return None
    gy, gx = np.nonzero(valid)
    idx = -np.ones((H, Wc), int)
    idx[gy, gx] = np.arange(len(gy))
    verts = geom.to_world(x0 + (gx + 0.5) * step, y0 + (gy + 0.5) * step, z[gy, gx].astype(float))
    # two triangles per square of four cells, where the cells are joined (no depth jump)
    a, b, c, e = idx[:-1, :-1], idx[:-1, 1:], idx[1:, :-1], idx[1:, 1:]
    za, zb, zc, ze = z[:-1, :-1], z[:-1, 1:], z[1:, :-1], z[1:, 1:]
    faces = []
    for (p, q, r), (zp, zq, zr) in (((a, b, c), (za, zb, zc)), ((b, e, c), (zb, ze, zc))):
        zs = np.stack([zp, zq, zr])
        with np.errstate(invalid="ignore"):  # cells without depth are inf
            good = (p >= 0) & (q >= 0) & (r >= 0) & (np.ptp(zs, axis=0) <= JOIN_MM + 0.03 * zs.min(axis=0))
        faces.append(np.stack([p[good], q[good], r[good]], axis=1))
    faces = np.concatenate(faces)
    # outline at the depth of the nearest cells
    pts = outline[::max(1, len(outline) // 64)]
    cc = np.column_stack([(gx + 0.5) * step, (gy + 0.5) * step])
    near = np.argsort(np.linalg.norm(pts[:, None] - cc[None], axis=2), axis=1)[:, :3]
    zo = np.median(z[gy, gx][near], axis=1)
    out3 = geom.to_world(x0 + pts[:, 0].astype(float), y0 + pts[:, 1].astype(float), zo.astype(float))
    # size: the smallest rectangle around the surface, in its own plane
    mc, size = verts.mean(axis=0), _plane_size(verts)
    return {"verts": verts, "faces": faces, "outline": out3, "centre": mc, "size_mm": [float(v) for v in size],
            "shape": _named(_shape(outline), size), "holes": 1 - np.count_nonzero(blob) / max(1, np.count_nonzero(filled)),
            "pockets": _pockets(blob, filled, x0, y0, depth, geom) if size[1] >= 2 * MIN_POCKET_MM else [],
            "pixels": int(np.count_nonzero(blob)), "img_outline": outline + [x0, y0]}


def _named(shape, size):
    """Shape name, with the true proportions (size, mm) for the ones perspective distorts."""
    if shape == "quadrilateral":
        return "square" if size[1] > 0.8 * size[0] else "rectangle"
    if shape in ("circle", "ellipse"):
        return "circle" if size[1] > 0.85 * size[0] else "ellipse"
    return shape


def _plane_size(P):
    """Sides (mm, longest first) of the smallest rectangle around points P, in their own plane."""
    m = P.mean(axis=0)
    ax = np.linalg.eigh(np.cov(P - m, rowvar=False))[1][:, ::-1]
    return sorted((float(v) for v in cv2.minAreaRect(((P - m) @ ax[:, :2]).astype(np.float32))[1]), reverse=True)


def _pockets(blob, filled, x0, y0, depth, geom):
    """Pockets (shaped holes) in an object: [{shape, size_mm, depth_mm, outline (at the rim, mm), img_outline}],
    largest first. Heights above the object's plane (least squares, so tilt doesn't matter) are closed
    morphologically over POCKET_SPAN_MM, which fills every pocket up to its rim; pockets are where that is more than
    POCKET_MM above the actual height (or where the outline holds another colour, e.g. a piece sitting in its
    pocket), not reaching the object's edge (the image border may cut them). Their outlines are where the camera
    rays through their contour meet the rim's plane."""
    hgt, wid = blob.shape
    d = depth[y0:y0 + hgt, x0:x0 + wid].astype(float)
    ok = filled & (d >= DEPTH_RANGE_MM[0]) & (d <= DEPTH_RANGE_MM[1])
    vs, us = np.nonzero(ok)
    if len(us) < 50:
        return []
    W = geom.to_world(x0 + us.astype(float), y0 + vs.astype(float), d[vs, us])
    c = W.mean(axis=0)
    n = np.linalg.eigh(np.cov(W - c, rowvar=False))[1][:, 0]
    if n @ (geom.centre - c) < 0:
        n = -n  # pointing to the camera: up out of the object
    H = np.zeros(blob.shape, np.float32)
    H[vs, us] = (W - c) @ n
    # depth holes inside the outline take the nearest height
    _, near = cv2.distanceTransformWithLabels((~ok).astype(np.uint8), cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
    src = np.zeros((near.max() + 1, 2), int)
    src[near[ok]] = np.column_stack([vs, us])
    H = H[src[near][..., 0], src[near][..., 1]]
    k = int(min(151, POCKET_SPAN_MM * geom.fx / np.median(d[vs, us]))) | 1
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    lid = cv2.erode(np.where(filled, cv2.dilate(np.where(filled, H, -1e4).astype(np.float32), ker), 1e4).astype(np.float32), ker)
    below = lid - H
    pocket = filled & ((below > POCKET_MM) | ~blob)
    pocket = cv2.morphologyEx(pocket.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    # the object's own edge (not where the image border cuts it)
    edge = filled.astype(np.uint8) - cv2.erode(filled.astype(np.uint8), np.ones((3, 3), np.uint8))
    yy, xx = np.nonzero(edge)
    on_border = (x0 + xx <= 1) | (y0 + yy <= 1) | (x0 + xx >= geom.w - 2) | (y0 + yy >= geom.h - 2)
    edge[yy[on_border], xx[on_border]] = 0
    edge = cv2.dilate(edge, np.ones((7, 7), np.uint8)) > 0
    n_c, lab, stats, _ = cv2.connectedComponentsWithStats(pocket)
    rays = lambda px: np.column_stack([(px[:, 0] - geom.ppx) / geom.fx, (px[:, 1] - geom.ppy) / geom.fy,
                                       np.ones(len(px))]) @ geom.R.T
    out = []
    for i in np.argsort(-stats[1:, cv2.CC_STAT_AREA]) + 1:
        part = lab == i
        if np.any(part & edge):
            continue  # reaches the object's edge: its sides, not a pocket
        cs, _ = cv2.findContours(part.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnt = max(cs, key=cv2.contourArea).reshape(-1, 2)
        if cv2.contourArea(cnt) < 0.85 * cv2.contourArea(cv2.convexHull(cnt)):
            continue  # pockets are convex shapes; this is noise (or pockets run together)
        px = (cnt[::max(1, len(cnt) // 48)] + [x0, y0]).astype(float)
        r = rays(px)
        rim = c + float(np.median(lid[part])) * n
        P = geom.centre + ((rim - geom.centre) @ n / (r @ n))[:, None] * r
        size = _plane_size(P)
        if size[1] < MIN_POCKET_MM:
            continue
        out.append({"shape": _named(_shape(cnt), size), "size_mm": size, "outline": P,
                    "depth_mm": float(np.median(below[part & ok])) if np.count_nonzero(part & ok) > 10 else None,
                    "img_outline": cnt + [x0, y0]})
        if len(out) >= MAX_POCKETS:
            break
    return out


def _ransac(W, blob, rng):
    """Inliers of the line through the most points W. Candidate lines join two points of one image blob, and only
    count points of that blob (a rod split by a connector is two rods, merged again by RodTracker)."""
    order = np.argsort(blob, kind="stable")
    ids, first, count = np.unique(blob[order], return_index=True, return_counts=True)
    i = rng.integers(0, len(W), RANSAC_ITERS)
    b = np.searchsorted(ids, blob[i])
    j = order[first[b] + (rng.random(RANSAC_ITERS) * count[b]).astype(int)]
    d = W[j] - W[i]
    nd = np.linalg.norm(d, axis=1)
    keep = nd >= 10
    i, d = i[keep], d[keep] / nd[keep, None]
    best, best_n = None, 0
    for k in range(0, len(i), 30):  # 30 candidate lines at a time
        r = W[None] - W[i[k:k + 30], None]
        along = np.einsum("cnk,ck->cn", r, d[k:k + 30])
        inl = np.einsum("cnk,cnk->cn", r, r) - along ** 2 < INLIER_MM ** 2
        inl &= blob[None] == blob[i[k:k + 30], None]  # a line doesn't jump between blobs
        n = inl.sum(axis=1)
        if n.max() > best_n:
            best, best_n = inl[n.argmax()], n.max()
    return best


def _find_rods(mask, depth, geom, rng):
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    small = stats[:, cv2.CC_STAT_AREA] < MIN_BLOB_PX
    small[0] = True
    vs, us = np.nonzero(~small[lab] & (depth >= DEPTH_RANGE_MM[0]) & (depth <= DEPTH_RANGE_MM[1]))
    if len(us) < MIN_PIXELS // 2:
        return []
    k = min(len(us), SAMPLE)
    per = len(us) / k  # image pixels per sampled depth point
    pick = rng.choice(len(us), k, replace=False)
    us, vs = us[pick], vs[pick]
    z = depth[vs, us].astype(float)
    W = geom.to_world(us.astype(float), vs.astype(float), z)
    blob = lab[vs, us]
    my, mx = np.nonzero(mask)
    pix = np.column_stack([mx, my]).astype(float)  # all pixels of this colour, for the rods' widths
    rods, left = [], np.ones(k, bool)
    for _ in range(2 * MAX_RODS):
        idx = np.flatnonzero(left)
        if len(rods) >= MAX_RODS or len(idx) * per < MIN_PIXELS // 2:
            break
        inl = _ransac(W[idx], blob[idx], rng)
        if inl is None or inl.sum() * per < MIN_PIXELS // 2:
            break
        idx = idx[inl]
        # the line may run through several rods in a row: take the biggest stretch without a gap
        m, u = _axis(W[idx])
        s = (W[idx] - m) @ u
        o = np.argsort(s)
        run = idx[max(np.split(o, np.flatnonzero(np.diff(s[o]) > GAP_MM) + 1), key=len)]
        if len(run) * per < MIN_PIXELS // 2:
            left[idx] = False
            continue
        rod = _fit(np.column_stack([us[run], vs[run]]).astype(float), z[run], pix, mask, depth, geom, rng)
        # drop this rod's points, including those just outside the inlier distance (the rod's other faces)
        r = W - W[run].mean(axis=0)
        sr = (W[run] - W[run].mean(axis=0)) @ u
        sa = r @ u
        left &= ~((np.linalg.norm(r - np.outer(sa, u), axis=1) < 2 * INLIER_MM) &
                  (sa > sr.min() - INLIER_MM) & (sa < sr.max() + INLIER_MM))
        left[run] = False
        if rod is not None:
            # and the points on its whole length in the image at its depth, and behind it (the background's depth
            # leaking through it)
            o, d, a, b = rod.pop("depth_line")
            rel = np.column_stack([us, vs]) - o
            t = rel @ d
            on = (np.abs(rel @ [-d[1], d[0]]) < 1.5 * rod["half_px"] + 3) & (t > -2 * rod["half_px"]) & \
                (t < np.linalg.norm(rod["img"][1] - rod["img"][0]) + 2 * rod["half_px"])
            left &= ~(on & (z - 1 / np.maximum(a + b * t, 1e-9) > -3 * INLIER_MM))  # (and the depth behind it)
            rods.append({**rod, "pixels": int(np.count_nonzero(on) * per)})
    return rods


def _fit(uv, zr, pix, mask, depth, geom, rng):
    """Rod through the depth points at image pixels uv (on its visible surface), or None if too short, too thick or
    not a rod. A rod is only a few pixels wide and its depth is noisy, so a 3D line fitted to a short piece of it
    points off, and extended along the rod it ends up far away. Instead the line is fitted in the image (the colour
    is sharp), followed along the colour where depth is missing, and the depth along it is a line fitted to the
    depth of the rod's middle pixels (1/depth is linear along a line in the image)."""
    vx, vy, x0, y0 = cv2.fitLine(uv.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    o, d = np.array([x0, y0], float), np.array([vx, vy], float)
    t = (uv - o) @ d
    mm = geom.fx / float(np.median(zr))  # pixels per mm at the rod
    t0, t1 = np.percentile(t, [1, 99])
    t0, t1 = _follow(o, d, t0, -1, mask, mm), _follow(o, d, t1, 1, mask, mm)
    img = np.array([o + t0 * d, o + t1 * d])
    len_px = max(1.0, t1 - t0)
    nrm = np.array([-d[1], d[0]])
    # width in the image: rod-coloured pixels per pixel of length near the line, the median along it (so crossing
    # rods and gaps don't count); the radius at the rod's depth
    rel = pix - img[0]
    t = rel @ d
    side_off = rel @ nrm
    off = np.abs(side_off)
    along = (t >= 0) & (t < len_px)
    near = along & (off < MAX_ROD_MM / 2 * mm)
    if not near.any():
        return None
    counts = np.bincount(t[near].astype(int), minlength=int(len_px) + 1)
    slices = np.flatnonzero(counts)
    half = float(np.median(counts[slices])) / 2
    # at least one side of a rod is background (except where another rod crosses); inside a mesh or panel of the
    # rod's colour, neither is
    lo = 1.5 * half + 2
    side = (off >= lo) & (off < lo + FLANK_MM * mm)
    if min(np.median(np.bincount(t[along & side & ((side_off > 0) == k)].astype(int), minlength=int(len_px) + 1)[slices])
           for k in (False, True)) > MAX_FLANK * FLANK_MM * mm:
        return None
    # depth along the rod, from its middle pixels (its edges mix with the background)
    core = np.flatnonzero(along & (off <= max(1.0, 0.6 * half)))
    zc = depth[pix[core, 1].astype(int), pix[core, 0].astype(int)].astype(float)
    ok = (zc >= DEPTH_RANGE_MM[0]) & (zc <= DEPTH_RANGE_MM[1])
    # a thin glossy rod often has no depth of its own, and its pixels get the depth of what is behind it: the rod's
    # depth must be in front of the background beside it (on each side that has depth)
    h, w = depth.shape
    behind = np.ones(len(core), bool)
    for sgn in (-1, 1):
        far = np.zeros(len(core))
        for step in (lo + 1, lo + 3, lo + 5):
            q = np.rint(pix[core] + sgn * step * nrm).astype(int)
            inside = (q[:, 0] >= 0) & (q[:, 0] < w) & (q[:, 1] >= 0) & (q[:, 1] < h)
            zq = np.zeros(len(core))
            zq[inside] = depth[q[inside, 1], q[inside, 0]]
            zq[(zq < DEPTH_RANGE_MM[0]) | (zq > 4 * DEPTH_RANGE_MM[1])] = 0
            far = np.maximum(far, zq)
        behind &= (far == 0) | (far > zc + FRONT_MM)
    ok &= behind
    line = _depth_line(t[core][ok], zc[ok], rng)
    if line is None:
        return None
    a, b, tin = line
    # the depth slope is extrapolated beyond the stretch with depth: no further than the stretch is long
    lo_t, hi_t = np.percentile(tin, [2, 98])
    reach = max(hi_t - lo_t, 0.0)
    te = np.array([max(0.0, lo_t - reach), min(len_px, hi_t + reach)])
    img = img[0] + te[:, None] * d
    w = a + b * te
    if np.any(w <= 0):
        return None
    ze = 1 / w
    if np.any((ze < DEPTH_RANGE_MM[0]) | (ze > DEPTH_RANGE_MM[1])):
        return None
    ends = geom.to_world(img[:, 0], img[:, 1], ze)
    length = float(np.linalg.norm(ends[1] - ends[0]))
    if length < MIN_LENGTH_MM:
        return None
    radius = half / (geom.fx / float(np.median(1 / (a + b * tin))))
    if 2 * radius > MAX_ROD_MM or length < ROD_ASPECT * 2 * radius:
        return None  # too thick, or too stubby for a rod (a button, a blob)
    u = _unit(ends[1] - ends[0])
    view = (ends[0] + ends[1]) / 2 - geom.centre
    if abs(u @ view) > math.cos(math.radians(MAX_VIEW_DEG)) * np.linalg.norm(view):
        return None  # along the line of sight: depth smeared over an edge
    radius = max(2.0, radius)
    # the depth points are on the side facing the camera: move the axis away from the camera
    away = view - (view @ u) * u
    ends = ends + SURFACE_SHIFT * radius * away / np.linalg.norm(away)
    opened = [bool(p[0] < BORDER_PX or p[1] < BORDER_PX or p[0] > geom.w - 1 - BORDER_PX or p[1] > geom.h - 1 - BORDER_PX)
              for p in img]
    return {"p0": ends[0], "p1": ends[1], "radius": radius, "open": opened, "img": img, "half_px": half,
            "depth_line": (img[0], d, a, b)}


def _depth_line(t, z, rng):
    """Robust fit of 1/z = a + b t to depth z at positions t along a line in the image (RANSAC, INLIER_MM, then least
    squares on the inliers, weighted so the residuals are in mm). Returns (a, b, t of the inliers) or None."""
    if len(z) < MIN_DEPTH_PX:
        return None
    w = 1 / z
    i, j = rng.integers(0, len(z), (2, DEPTH_ITERS))
    keep = np.abs(t[j] - t[i]) > 3
    i, j = i[keep], j[keep]
    b = (w[j] - w[i]) / (t[j] - t[i])
    a = w[i] - b * t[i]
    # also the flat line through the median depth (a rod across the view, or depth on a short stretch only)
    a, b = np.append(a, 1 / np.median(z)), np.append(b, 0.0)
    pred = a[:, None] + b[:, None] * t[None]
    with np.errstate(divide="ignore"):
        res = z[None] - 1 / pred
    res[pred <= 0] = np.inf
    inl = np.abs(res) < INLIER_MM
    # depth leaking from behind the rod is further away, never nearer: a line with depth in front of it is wrong
    best = inl[(inl.sum(axis=1) - 2 * (res < -INLIER_MM).sum(axis=1)).argmax()]
    if best.sum() < MIN_DEPTH_PX:
        return None
    tb, wb, zb = t[best], w[best], z[best]
    A = np.column_stack([np.ones_like(tb), tb]) * (zb ** 2)[:, None]
    a, b = np.linalg.lstsq(A, wb * zb ** 2, rcond=None)[0]
    return a, b, tb


def _follow(o, d, t, sign, mask, mm):
    """Position along the image line o + t d beyond t (direction sign) up to which the line stays on the colour mask
    (over gaps of up to FOLLOW_GAP_MM): extends a rod over the part where its depth is missing."""
    h, w = mask.shape
    gap = max(2, int(round(FOLLOW_GAP_MM * mm)))
    last, miss = t, 0
    for k in range(1, 2 * (h + w)):
        tk = t + sign * k
        x, y = o + tk * d
        if not (0 <= x < w and 0 <= y < h):
            break
        xi, yi = int(x), int(y)
        if mask[max(0, yi - 1):yi + 2, max(0, xi - 1):xi + 2].any():
            last, miss = tk, 0
        else:
            miss += 1
            if miss > gap:
                break
    return last


def _axis(P):
    """Mean and direction (unit vector) of the line through points P (principal axis)."""
    m = P.mean(axis=0)
    return m, np.linalg.eigh(np.cov(P - m, rowvar=False))[1][:, -1]


def _unit(v):
    return v / np.linalg.norm(v)


class _Track:
    """One static rod, merged over detections: the axis is smoothed and the visible extent only grows."""

    SMOOTH = 0.15
    SAME_DEG, SAME_MM = 8.0, 10.0

    def __init__(self, det):
        p0, p1 = np.asarray(det["p0"], float), np.asarray(det["p1"], float)
        self.u, self.c = _unit(p1 - p0), (p0 + p1) / 2
        self.s = [(p0 - self.c) @ self.u, (p1 - self.c) @ self.u]
        self.r, self.open, self.colour = det["radius"], list(det["open"]), det["colour"]
        self.px, self.hits, self.misses = float(det["pixels"]), 1, 0

    def _offset(self, p0, p1):
        """(angle, distance) between the segment p0-p1's line and the track's axis."""
        ud = _unit(p1 - p0)
        md = (p0 + p1) / 2
        off = (self.c - md) - ((self.c - md) @ ud) * ud
        return math.degrees(math.acos(min(1.0, abs(ud @ self.u)))), np.linalg.norm(off)

    def matches(self, det):
        a, d = self._offset(np.asarray(det["p0"], float), np.asarray(det["p1"], float))
        return a < self.SAME_DEG and d < self.SAME_MM

    def add(self, det):
        p0, p1 = np.asarray(det["p0"], float), np.asarray(det["p1"], float)
        ud = _unit(p1 - p0)
        opened = det["open"]
        if ud @ self.u < 0:  # same orientation as the track
            p0, p1, ud, opened = p1, p0, -ud, opened[::-1]
        md = (p0 + p1) / 2
        off = (self.c - md) - ((self.c - md) @ ud) * ud
        k = self.SMOOTH
        self.u = _unit((1 - k) * self.u + k * ud)
        self.c = self.c - k * off  # pull the axis toward the new line
        self.r = (1 - k) * self.r + k * det["radius"]
        self.px = (1 - k) * self.px + k * det["pixels"]
        s = sorted([(p0 - self.c) @ self.u, (p1 - self.c) @ self.u])
        self.s = [min(self.s[0], s[0]), max(self.s[1], s[1])]
        self.open = [self.open[0] or opened[0], self.open[1] or opened[1]]
        self.hits, self.misses = self.hits + 1, 0

    def ends(self):
        return self.c + self.s[0] * self.u, self.c + self.s[1] * self.u

    def absorb(self, other):
        """Merge another track of the same rod into this one."""
        s = sorted((e - self.c) @ self.u for e in other.ends())
        self.s = [min(self.s[0], s[0]), max(self.s[1], s[1])]
        self.hits, self.misses = max(self.hits, other.hits), min(self.misses, other.misses)
        self.px = max(self.px, other.px)

    def overlaps(self, other):
        """Whether the other track lies along this one's axis (both its ends within both radii), overlapping its
        extent: a duplicate or a piece of this rod."""
        E = np.array(other.ends()) - self.c
        s = E @ self.u
        d = np.linalg.norm(E - np.outer(s, self.u), axis=1)
        return d.max() < self.r + other.r + INLIER_MM and s.max() > self.s[0] and s.min() < self.s[1]

    def rod(self):
        seen = self.s[1] - self.s[0]
        ext = min(EXTEND_MM, seen) if seen >= EXTEND_MIN_MM else 0.0
        s0 = self.s[0] - (ext if self.open[0] else 0.0)
        s1 = self.s[1] + (ext if self.open[1] else 0.0)
        return {"p0": (self.c + s0 * self.u).round(1).tolist(), "p1": (self.c + s1 * self.u).round(1).tolist(),
                "radius": round(float(self.r), 1), "open": self.open, "colour": self.colour,
                "visible_mm": round(float(self.s[1] - self.s[0]), 1)}


class RodTracker:
    """Tracks several static rods. A rod is reported once seen in CONFIRM detections, and forgotten when not seen
    for DROP_AFTER detections in a row (moved away). Of rods along the same line, only the biggest is reported
    (e.g. the orange-looking lit edge of a red bar)."""

    CONFIRM = 3
    DROP_AFTER = 40

    def __init__(self):
        self.reset()

    def reset(self):
        self.tracks = []

    def update(self, dets):
        for t in self.tracks:
            t.misses += 1
        for d in dets:
            t = next((t for t in self.tracks if t.colour == d["colour"] and t.matches(d)), None)
            if t is None:
                self.tracks.append(_Track(d))
            else:
                t.add(d)
        merged = []
        for t in sorted(self.tracks, key=lambda t: -t.hits):
            if t.misses >= (self.DROP_AFTER if t.hits >= self.CONFIRM else 3):
                continue
            k = next((k for k in merged if k.colour == t.colour and k.overlaps(t)), None)
            if k is None:
                merged.append(t)
            else:
                k.absorb(t)
        self.tracks = merged

    def rods(self):
        """The confirmed rods (open ends extended), biggest first."""
        out = []
        for t in sorted((t for t in self.tracks if t.hits >= self.CONFIRM), key=lambda t: -t.px * (t.s[1] - t.s[0])):
            if not any(k.overlaps(t) for k in out):
                out.append(t)
        return [t.rod() for t in out]


class ObjectTracker:
    """Tracks objects (matched by colour and centre). An object is reported once seen in CONFIRM detections in a
    row, and forgotten when not seen for DROP_AFTER detections in a row. Its shape is the most frequent one seen; its
    geometry is updated (and version increased) only when it moved or changed noticeably, not with every frame's
    noise."""

    CONFIRM, DROP_AFTER = 3, 40
    MOVED_MM, CHANGED = 5.0, 0.15

    def __init__(self):
        self.next_id, self.version = 1, 0
        self.reset()

    def reset(self):
        self.tracks, self._ids = [], ()
        self.version += 1

    def update(self, dets):
        for t in self.tracks:
            t["misses"] += 1
        for d in sorted(dets, key=lambda d: -d["pixels"]):
            dist = lambda t: np.linalg.norm(t["det"]["centre"] - d["centre"])
            near = [t for t in self.tracks if t["colour"] == d["colour"] and t["misses"] > 0 and
                    dist(t) < max(15.0, 0.4 * t["det"]["size_mm"][0])]
            if near:
                t = min(near, key=dist)
                t.update(det=d, hits=t["hits"] + 1, misses=0)
                t["shapes"][d["shape"]] += 1
            else:
                self.tracks.append({"id": self.next_id, "colour": d["colour"], "det": d, "shown": None, "hits": 1,
                                    "misses": 0, "shapes": collections.Counter([d["shape"]])})
                self.next_id += 1
        self.tracks = [t for t in self.tracks if t["misses"] < (self.DROP_AFTER if t["hits"] >= self.CONFIRM else 3)]
        for t in self.tracks:
            d, s = t["det"], t["shown"]
            if t["hits"] < self.CONFIRM or t["misses"]:
                continue
            if s is None or np.linalg.norm(s["centre"] - d["centre"]) > self.MOVED_MM or \
                    abs(d["pixels"] - s["pixels"]) > self.CHANGED * s["pixels"]:
                t["shown"] = d
                self.version += 1
        ids = tuple((t["id"], t["shapes"].most_common(1)[0][0]) for t in self.tracks if t["shown"] is not None)
        if ids != self._ids:
            self._ids = ids
            self.version += 1

    def objects(self):
        """The confirmed objects: id, colour, shape, size_mm, centre, holes, verts, faces, outline (mm), pockets
        and img_outline (latest image contour, for the live view; also per pocket)."""
        out = []
        for t in self.tracks:
            s = t["shown"]
            if s is None:
                continue
            out.append({"id": t["id"], "colour": t["colour"], "shape": t["shapes"].most_common(1)[0][0],
                        "size_mm": [round(v, 1) for v in s["size_mm"]], "centre": s["centre"].round(1).tolist(),
                        "holes": round(float(s["holes"]), 2), "verts": s["verts"].round(1).tolist(),
                        "faces": s["faces"].tolist(), "outline": s["outline"].round(1).tolist(),
                        "pockets": [{"shape": p["shape"], "size_mm": [round(v, 1) for v in p["size_mm"]],
                                     "depth_mm": None if p["depth_mm"] is None else round(p["depth_mm"], 1),
                                     "outline": p["outline"].round(1).tolist(), "img_outline": p["img_outline"]}
                                    for p in s["pockets"]],
                        "img_outline": t["det"]["img_outline"]})
        return out


DRAW_BGR = {"red": (0, 0, 255), "orange": (0, 140, 255), "yellow": (0, 220, 255), "pink": (180, 105, 255),
            "purple": (200, 60, 150), "green": (60, 160, 40)}


def draw(frame, masks, dets, rods, geom, objects=()):
    """Live-view overlay: the colour masks' outlines, this frame's fitted rods (thin), the tracked rods and the
    tracked objects' outlines with their shape."""
    for mask in masks.values():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (255, 0, 255), 1)
    for det in dets:
        a, b = (tuple(int(round(c)) for c in e) for e in det["img"])
        cv2.line(frame, a, b, (0, 255, 255), 1, cv2.LINE_AA)
    for rod in rods:
        ends = geom.to_image(np.array([rod["p0"], rod["p1"]]))
        if not np.all(np.isfinite(ends)):
            continue
        a, b = (tuple(int(round(c)) for c in e) for e in ends)
        cv2.line(frame, a, b, DRAW_BGR.get(rod["colour"], (255, 0, 255)), 2, cv2.LINE_AA)
        m = ((a[0] + b[0]) // 2 + 5, (a[1] + b[1]) // 2 - 5)
        cv2.putText(frame, f"{rod['colour']} r={rod['radius']:.0f}", m, cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 0, 255), 1, cv2.LINE_AA)
    for obj in objects:
        c = obj["img_outline"].reshape(-1, 1, 2).astype(np.int32)
        cv2.polylines(frame, [c], True, DRAW_BGR.get(obj["colour"], (255, 0, 255)), 2, cv2.LINE_AA)
        x, y = c[:, 0].min(axis=0)
        cv2.putText(frame, f"{obj['shape']} {obj['size_mm'][0]:.0f} mm", (int(x), max(12, int(y) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)
        for p in obj.get("pockets", []):
            pc = p["img_outline"].reshape(-1, 1, 2).astype(np.int32)
            cv2.polylines(frame, [pc], True, (255, 255, 0), 1, cv2.LINE_AA)
            px, py = pc[:, 0].mean(axis=0)
            cv2.putText(frame, f"{p['shape']} pocket", (int(px) - 25, int(py)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (255, 255, 0), 1, cv2.LINE_AA)
