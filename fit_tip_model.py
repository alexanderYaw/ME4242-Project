#!/usr/bin/env python3
"""Fit the tip model (bend values dA, dB -> tip position) from a collect_tip_data.py CSV.

Default form "cc" (constant curvature, physical): the arm bends as a circular arc from its attachment point. Each
motor adds bend (an angle, growing quadratically with its pull) toward its own direction; the bends add as vectors
(what the cable geometry of a constant-curvature arm gives), and the tip lies at the end of the arc. Reach therefore
saturates like the real arm's: past ~120 deg of bend the tip mostly rises instead of moving out, so pulling two
adjacent motors fully doesn't reach much further out than one. Fitted parameters: attachment point, arm length, a
small rest bend, and per motor its direction and pull->bend curve.

Form "per_motor": the tip is a sum of each motor's own effect, a quadratic in that motor's pull (dA>0 is
motor 0, dA<0 motor 2, dB>0 motor 1, dB<0 motor 3), lightly ridge-regularised. It is continuous across the motor
axes by construction and does not fold, so the GUI's tip control can move the target smoothly everywhere.
--form quadrant: the older separate quadratic per quadrant (slightly more accurate where data is dense, but the
quadrants don't meet at the axes and it can fold where data is sparse). It also estimates where the arm is attached
(base) and its length, assuming a constant-curvature arc that leaves the base pointing straight down; the GUI
uses that to draw the arm.

--mirror 1,2: those motors' cables weren't connected during collection. Their terms are left out of the fit and
filled in from the opposite motor's (0<->2, 1<->3) response, rotated 180 deg about the vertical axis. A guess that
assumes the arm is symmetric; recollect with all cables for a real fit.

Usage:  ~/emio-venv/bin/python fit_tip_model.py data/tip_<time>.csv [--out motor_gui/data/tip_model.json]
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np

DEG = 2
QUADRANTS = {"++": (1, 1), "+-": (1, -1), "-+": (-1, 1), "--": (-1, -1)}
PER_MOTOR_TERMS = ["1", "a+", "a-", "b+", "b-", "a+^2", "a-^2", "b+^2", "b-^2"]
RIDGE = 0.05
CC_MAX_BEND_RAD = 2.1  # tip control stays below this bend: beyond ~2.33 rad the tip curls back inward
MOTOR_TERMS = {0: (1, 5), 2: (2, 6), 1: (3, 7), 3: (4, 8)}  # per_motor term indices: (linear, squared)
OPPOSITE = {0: 2, 2: 0, 1: 3, 3: 1}
ROT180 = np.diag([-1.0, 1.0, -1.0])  # 180 deg about the vertical (y) axis


def per_motor_features(D):
    a, b = D[:, 0], D[:, 1]
    ap, an, bp, bn = np.maximum(a, 0), np.minimum(a, 0), np.maximum(b, 0), np.minimum(b, 0)
    return np.column_stack([np.ones_like(a), ap, an, bp, bn, ap ** 2, an ** 2, bp ** 2, bn ** 2])


def fit_per_motor(D, P, ridge=RIDGE, mirror=()):
    X = per_motor_features(D)
    keep = [j for j in range(X.shape[1]) if not any(j in MOTOR_TERMS[m] for m in mirror)]
    R = ridge * np.eye(len(keep))
    R[0, 0] = 0  # don't shrink the rest position
    W = np.zeros((X.shape[1], 3))
    W[keep] = np.linalg.solve(X[:, keep].T @ X[:, keep] + R, X[:, keep].T @ P)
    for m in mirror:
        (lin, sq), (src_lin, src_sq) = MOTOR_TERMS[m], MOTOR_TERMS[OPPOSITE[m]]
        W[lin] = -W[src_lin] @ ROT180  # the linear feature's sign flips between opposite motors
        W[sq] = W[src_sq] @ ROT180
    return W.tolist()


def predict_per_motor(W, D):
    return per_motor_features(D) @ np.array(W)


def motor_pulls(D):
    """Pull fraction (0..1) of each motor 0..3 for bend values (dA, dB)."""
    a, b = D[:, 0], D[:, 1]
    return np.column_stack([np.maximum(a, 0), np.maximum(b, 0), np.maximum(-a, 0), np.maximum(-b, 0)])


def cc_bend(c, D):
    """Bend vector (x, z components, rad) for bend values D under constant-curvature params c."""
    p = motor_pulls(D)
    beta = np.tile(np.array(c["bend0"], dtype=float), (len(D), 1))
    for i, m in enumerate(c["motors"]):
        g = m["c1"] * p[:, i] + m["c2"] * p[:, i] ** 2
        beta += g[:, None] * np.array([math.cos(m["phi"]), math.sin(m["phi"])])
    return beta


def predict_cc(c, D):
    beta = cc_bend(c, D)
    th = np.hypot(beta[:, 0], beta[:, 1])
    L = c["length_mm"]
    small = th < 1e-6
    ths = np.where(small, 1.0, th)
    h = np.where(small, L * th / 2, L * (1 - np.cos(ths)) / ths)  # horizontal reach
    v = np.where(small, L, L * np.sin(ths) / ths)                 # depth below the attachment
    ux, uz = np.where(small, 1.0, beta[:, 0] / ths), np.where(small, 0.0, beta[:, 1] / ths)
    bx, by, bz = c["base"]
    return np.column_stack([bx + h * ux, by - v, bz + h * uz])


def _cc_unpack(x, fitted):
    c = {"base": x[0:3].tolist(), "length_mm": float(x[3]), "bend0": x[4:6].tolist(),
         "motors": [{"phi": 0.0, "c1": 0.0, "c2": 0.0} for _ in range(4)]}
    for k, i in enumerate(fitted):
        phi, c1, c2 = x[6 + 3 * k: 9 + 3 * k]
        c["motors"][i] = {"phi": float(phi), "c1": float(c1), "c2": float(c2)}
    return c


def fit_cc(D, P, mirror=()):
    """Levenberg-Marquardt fit of the constant-curvature model. Mirrored motors (cables not connected) are held at
    zero effect during the fit, then copied from their opposite motor with the direction turned 180 deg."""
    fitted = [i for i in range(4) if i not in mirror]
    # start: attachment above the rest tip, each motor bending toward where its own pulls moved the tip
    rest = P[np.argmin(np.hypot(D[:, 0], D[:, 1]))]
    x = [rest[0], rest[1] + 195.0, rest[2], 195.0, 0.0, 0.0]
    pulls = motor_pulls(D)
    for i in fitted:
        alone = (pulls[:, i] > 0.4) & (np.delete(pulls, i, axis=1).max(axis=1) < 1e-9)
        dxz = (P[alone][:, [0, 2]] - rest[[0, 2]]).mean(axis=0) if alone.any() else np.array([1.0, 0.0])
        x += [math.atan2(dxz[1], dxz[0]), 1.5, 0.0]
    x = np.array(x, dtype=float)

    def res(x):
        return (predict_cc(_cc_unpack(x, fitted), D) - P).ravel()

    r, lam = res(x), 1e-2
    for _ in range(300):
        J = np.empty((len(r), len(x)))
        for k in range(len(x)):
            e = np.zeros_like(x)
            e[k] = 1e-6 * max(1.0, abs(x[k]))
            J[:, k] = (res(x + e) - r) / e[k]
        A, g = J.T @ J, J.T @ r
        while True:
            dx = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -g)
            r_new = res(x + dx)
            if r_new @ r_new < r @ r:
                improved = (r @ r - r_new @ r_new) / (r @ r)
                x, r, lam = x + dx, r_new, max(lam / 3, 1e-9)
                break
            lam *= 4
            if lam > 1e9:
                improved = 0
                break
        if improved < 1e-10:
            break
    c = _cc_unpack(x, fitted)
    for m in mirror:
        src = c["motors"][OPPOSITE[m]]
        c["motors"][m] = {"phi": src["phi"] + math.pi, "c1": src["c1"], "c2": src["c2"]}
    for m in c["motors"]:
        m["phi"] = math.atan2(math.sin(m["phi"]), math.cos(m["phi"]))
    return c


def features(D, deg=DEG):
    a, b = D[:, 0], D[:, 1]
    return np.column_stack([a ** i * b ** j for i in range(deg + 1) for j in range(deg + 1 - i)])


def in_quadrant(D, sa, sb):
    return (D[:, 0] * sa >= -1e-9) & (D[:, 1] * sb >= -1e-9)


def fit(D, P):
    coeffs = {}
    for q, (sa, sb) in QUADRANTS.items():
        m = in_quadrant(D, sa, sb)
        W, *_ = np.linalg.lstsq(features(D[m]), P[m], rcond=None)
        coeffs[q] = W.tolist()
    return coeffs


def predict(coeffs, D):
    out = np.zeros((len(D), 3))
    for q, (sa, sb) in QUADRANTS.items():
        m = in_quadrant(D, sa, sb)
        out[m] = features(D[m]) @ np.array(coeffs[q])
    return out


def arc_lengths(P, base_xz, base_y):
    """Arc length from a base pointing straight down to each tip, for a constant-curvature arc."""
    h = np.hypot(P[:, 0] - base_xz[0], P[:, 2] - base_xz[1])
    v = base_y - P[:, 1]
    theta = 2 * np.arctan2(h, v)                      # bend angle
    chord = np.hypot(h, v)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.where(theta > 1e-6, chord * (theta / 2) / np.sin(theta / 2), chord)
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "motor_gui" / "data" / "tip_model.json"))
    ap.add_argument("--form", choices=("cc", "per_motor", "quadrant"), default="cc")
    ap.add_argument("--min-frames", type=int, default=5,
                    help="skip samples where the tip was seen in fewer camera frames (likely stale or false)")
    ap.add_argument("--mirror", default="", help="motors whose cables weren't connected, e.g. 1,2 (per_motor only)")
    args = ap.parse_args()
    mirror = tuple(int(m) for m in args.mirror.split(",") if m.strip())
    if mirror and (args.form == "quadrant" or any(OPPOSITE[m] in mirror for m in mirror)):
        ap.error("--mirror needs --form cc or per_motor, and each mirrored motor's opposite must have been connected")
    fit_fn, pred_fn = {
        "cc": ((lambda D, P: fit_cc(D, P, mirror=mirror)), predict_cc),
        "per_motor": ((lambda D, P: fit_per_motor(D, P, mirror=mirror)), predict_per_motor),
        "quadrant": (fit, predict),
    }[args.form]

    rows = [r for r in csv.DictReader(open(args.csv)) if r["tip_x"]]
    n_seen = len(rows)
    rows = [r for r in rows if int(float(r.get("tip_frames") or args.min_frames)) >= args.min_frames]
    if len(rows) < n_seen:
        print(f"skipped {n_seen - len(rows)} sample(s) seen in fewer than {args.min_frames} frames")
    D = np.array([[float(r["dA"]), float(r["dB"])] for r in rows])
    P = np.array([[float(r["tip_x"]), float(r["tip_y"]), float(r["tip_z"])] for r in rows])

    # 10-fold cross-validation for an honest error estimate
    idx = np.random.default_rng(0).permutation(len(D))
    errs = []
    for k in range(10):
        te, tr = idx[k::10], np.setdiff1d(idx, idx[k::10])
        errs.append(np.linalg.norm(pred_fn(fit_fn(D[tr], P[tr]), D[te]) - P[te], axis=1))
    e = np.concatenate(errs)
    if mirror:  # score only samples where the unconnected motors weren't pulling (the model says they'd bend it)
        pulling = {0: D[:, 0] > 1e-9, 2: D[:, 0] < -1e-9, 1: D[:, 1] > 1e-9, 3: D[:, 1] < -1e-9}
        scored = ~np.any([pulling[m] for m in mirror], axis=0)
        e = np.concatenate([err[~np.any([pulling[m][idx[k::10]] for m in mirror], axis=0)]
                            for k, err in enumerate(errs)])
        print(f"held-out error scored on the {scored.sum()} samples where motors {args.mirror} weren't pulling")
    coeffs = fit_fn(D, P)

    rest = pred_fn(coeffs, np.zeros((1, 2)))[0]
    if args.form == "cc":  # attachment and length are fitted parameters
        base_xz, base_y, length = (coeffs["base"][0], coeffs["base"][2]), coeffs["base"][1], coeffs["length_mm"]
        spread = [float(np.std(arc_lengths(P, base_xz, base_y)))]
    else:
        base_xz = (float(rest[0]), float(rest[2]))
        candidates = np.arange(rest[1] + 30, rest[1] + 500, 1.0)
        spread = [np.std(arc_lengths(P, base_xz, by)) for by in candidates]
        base_y = float(candidates[int(np.argmin(spread))])
        length = float(np.mean(arc_lengths(P, base_xz, base_y)))

    model = {
        "type": {"cc": "cc", "per_motor": "per_motor", "quadrant": "quadrant_poly"}[args.form], "degree": DEG,
        "coeffs": coeffs, "max_bend_rad": CC_MAX_BEND_RAD if args.form == "cc" else None,
        "terms": PER_MOTOR_TERMS if args.form == "per_motor" else None, "ridge": RIDGE if args.form == "per_motor" else None,
        "base": [base_xz[0], base_y, base_xz[1]], "length_mm": length,
        "rest_tip": rest.tolist(),
        "samples": P[:, [0, 2]].round(1).tolist(),
        "cv_error_mm": {"median": float(np.median(e)), "p90": float(np.percentile(e, 90)), "max": float(e.max())},
        "mirrored_motors": list(mirror), "source": str(Path(args.csv).resolve()), "fitted": time.strftime("%Y-%m-%d %H:%M:%S"), "n": len(D),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(model, indent=1))
    print(f"{len(D)} samples -> {args.out}")
    if mirror:
        print(f"motors {', '.join(map(str, mirror))} estimated by mirroring their opposite motors (not measured)")
    if args.form == "cc":
        for i, m in enumerate(coeffs["motors"]):
            full = m["c1"] + m["c2"]
            mono = m["c1"] >= 0 and m["c1"] + 2 * m["c2"] >= 0
            print(f"  motor {i}: bends toward {math.degrees(m['phi']):6.1f} deg (from +x toward +z), "
                  f"{math.degrees(full):5.1f} deg at full pull" + ("" if mono else "  (WARNING: not monotonic)"))
    print(f"held-out tip error: median {model['cv_error_mm']['median']:.1f} mm, "
          f"90% {model['cv_error_mm']['p90']:.1f} mm, max {model['cv_error_mm']['max']:.1f} mm")
    print(f"rest tip ({rest[0]:.1f}, {rest[1]:.1f}, {rest[2]:.1f}); estimated base y {base_y:.0f} mm, "
          f"arm length {length:.0f} mm (arc-length spread {min(spread):.1f} mm)")


if __name__ == "__main__":
    main()
