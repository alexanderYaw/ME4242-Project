#!/usr/bin/env python3
"""Fit the tip model (bend values dA, dB -> tip position) from a collect_tip_data.py CSV.

The arm switches motors at dA = 0 / dB = 0, so the model is a separate quadratic polynomial per quadrant
(sign of dA, sign of dB); neighbouring quadrants share the axes. It also estimates where the arm is attached
(base) and its length, assuming a constant-curvature arc that leaves the base pointing straight down; the GUI
uses that to draw the arm.

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
    args = ap.parse_args()

    rows = [r for r in csv.DictReader(open(args.csv)) if r["tip_x"]]
    D = np.array([[float(r["dA"]), float(r["dB"])] for r in rows])
    P = np.array([[float(r["tip_x"]), float(r["tip_y"]), float(r["tip_z"])] for r in rows])

    # 10-fold cross-validation for an honest error estimate
    idx = np.random.default_rng(0).permutation(len(D))
    errs = []
    for k in range(10):
        te, tr = idx[k::10], np.setdiff1d(idx, idx[k::10])
        errs.append(np.linalg.norm(predict(fit(D[tr], P[tr]), D[te]) - P[te], axis=1))
    e = np.concatenate(errs)
    coeffs = fit(D, P)

    rest = predict(coeffs, np.zeros((1, 2)))[0]
    base_xz = (float(rest[0]), float(rest[2]))
    candidates = np.arange(rest[1] + 30, rest[1] + 500, 1.0)
    spread = [np.std(arc_lengths(P, base_xz, by)) for by in candidates]
    base_y = float(candidates[int(np.argmin(spread))])
    length = float(np.mean(arc_lengths(P, base_xz, base_y)))

    model = {
        "type": "quadrant_poly", "degree": DEG, "coeffs": coeffs,
        "base": [base_xz[0], base_y, base_xz[1]], "length_mm": length,
        "rest_tip": rest.tolist(),
        "samples": P[:, [0, 2]].round(1).tolist(),
        "cv_error_mm": {"median": float(np.median(e)), "p90": float(np.percentile(e, 90)), "max": float(e.max())},
        "source": str(Path(args.csv).resolve()), "fitted": time.strftime("%Y-%m-%d %H:%M:%S"), "n": len(D),
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(model, indent=1))
    print(f"{len(D)} samples -> {args.out}")
    print(f"held-out tip error: median {model['cv_error_mm']['median']:.1f} mm, "
          f"90% {model['cv_error_mm']['p90']:.1f} mm, max {model['cv_error_mm']['max']:.1f} mm")
    print(f"rest tip ({rest[0]:.1f}, {rest[1]:.1f}, {rest[2]:.1f}); estimated base y {base_y:.0f} mm, "
          f"arm length {length:.0f} mm (arc-length spread {min(spread):.1f} mm)")


if __name__ == "__main__":
    main()
