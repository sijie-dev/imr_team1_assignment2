import json
import os
from pathlib import Path

import numpy as np

from model_uos_imr import ActuatorConfiguration, rigid_body_kinematics
from math_uos_imr import Vector

HERE = Path(__file__).resolve().parent


def first_existing(*candidates):
    for c in candidates:
        if c is not None and Path(c).exists():
            return Path(c)
    return Path(candidates[-1])


# data folders: environment variable, then the submission layout (log/), then the original project
A1_DIR = first_existing(os.environ.get("A1_DIR"), HERE.parent / "log" / "assignment1",
                        Path.home() / "Desktop" / "Assignment 1" / "code and log")
A2_DIR = first_existing(os.environ.get("A2_DIR"), HERE.parent / "log", HERE.parent / "logs")

# physical Assignment 1 runs (same files as calc_Q.py); found either in Test N/ subfolders or flat
A1_NAMES = ["Test 1/1433.json", "Test 2/1437.json", "Test 3/1438.json",
            "Test 4/1440.json", "Test 5/1441.json"]
A1_LOGS = [A1_DIR / n if (A1_DIR / n).exists() else A1_DIR / Path(n).name for n in A1_NAMES]
STATIONARY_LOGS = [A2_DIR / f"stationary{i}.json" for i in range(1, 6)]

ddrive = ActuatorConfiguration(0.081, 0.074)  # same actuator model as laptop.py

LOOP_DT = 0.1             # laptop.py runs at 10 Hz -> Q is added every 0.1 s
SUB_DT = 0.02             # integration sub-step [s]
T_MIN, T_MAX = 0.4, 3.0   # usable length of one ArUco interval [s]
MIN_MOTION = 0.01         # robot must travel >= 1 cm between the two fixes
FLIP_POS, FLIP_YAW = 0.10, np.deg2rad(20)   # larger errors are ArUco flips -> dropped
GLITCH_POS, GLITCH_YAW = 0.05, np.deg2rad(10)  # stationary outliers -> dropped

# values used in laptop.py (from calc_Q.py), for comparison only
Q_NE_FIT, Q_G_FIT = 2.42e-05, 2.09e-05


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def quat_yaw(o):
    return 2 * np.arctan2(o["z"], o["w"])


def load_log(path):
    """Return aruco [t, N, E, yaw] and wheel speeds [t, right, left] arrays."""
    aruco, wheels = [], []
    with open(path) as f:
        for line in f:
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            topic, t_log, m = d["topic_name"], float(d["timestamp"]), d["message"]
            if topic == "/aruco":
                t = m["header"]["stamp"]
                t -= round((t - t_log) / 3600.0) * 3600.0   # remove whole-hour clock offset
                p = m["pose"]
                aruco.append([t, p["position"]["x"], p["position"]["y"],
                              quat_yaw(p["orientation"])])
            elif topic == "/true_wheel_speeds":
                wheels.append([t_log, m["vector"]["x"], m["vector"]["y"]])
    aruco, wheels = np.array(aruco), np.array(wheels)
    return aruco[np.argsort(aruco[:, 0])], wheels[np.argsort(wheels[:, 0])]


def predict(p0, t0, t1, wheels):
    """Propagate pose p0 from t0 to t1 with the measured wheel speeds."""
    p = Vector(3)
    p[0], p[1], p[2] = p0
    t, travelled = t0, 0.0
    while t < t1 - 1e-9:
        dt = min(SUB_DT, t1 - t)
        i = np.searchsorted(wheels[:, 0], t, side="right") - 1
        q = Vector(2)
        q[0], q[1] = (wheels[i, 1], wheels[i, 2]) if i >= 0 else (0.0, 0.0)
        u = ddrive.fwd_kinematics(q)
        p = rigid_body_kinematics(p, u, dt)
        travelled += abs(u[0, 0]) * dt
        t += dt
    return np.array([p[0, 0], p[1, 0], p[2, 0]]), travelled


def estimate_R():
    """Sample variance of stationary ArUco readings (N, E, yaw), averaged over logs."""
    var_n, var_e, var_g = [], [], []
    for path in STATIONARY_LOGS:
        if not path.exists():
            continue
        aruco, _ = load_log(path)
        n, e, g = aruco[:, 1], aruco[:, 2], aruco[:, 3]
        g = np.median(g) + wrap(g - np.median(g))
        keep = ((np.abs(n - np.median(n)) < GLITCH_POS) &
                (np.abs(e - np.median(e)) < GLITCH_POS) &
                (np.abs(wrap(g - np.median(g))) < GLITCH_YAW))
        var_n.append(np.var(n[keep], ddof=1))
        var_e.append(np.var(e[keep], ddof=1))
        var_g.append(np.var(g[keep], ddof=1))
    return np.mean(var_n), np.mean(var_e), np.mean(var_g)


def pair_errors(path):
    """Errors between consecutive ArUco fixes and kinematic predictions: rows [n, eN, eE, eg]."""
    aruco, wheels = load_log(path)
    rows = []
    for k in range(len(aruco) - 1):
        t0, t1 = aruco[k, 0], aruco[k + 1, 0]
        T = t1 - t0
        if not (T_MIN <= T <= T_MAX) or t0 < wheels[0, 0]:
            continue
        if np.all(aruco[k + 1, 1:] == aruco[k, 1:]):
            continue  # stale frame (same pose re-sent)
        pred, travelled = predict(aruco[k, 1:], t0, t1, wheels)
        if travelled < MIN_MOTION:
            continue  # robot not moving
        eN, eE = aruco[k + 1, 1] - pred[0], aruco[k + 1, 2] - pred[1]
        eg = wrap(aruco[k + 1, 3] - pred[2])
        if np.hypot(eN, eE) > FLIP_POS or abs(eg) > FLIP_YAW:
            continue  # ArUco flip
        rows.append([T / LOOP_DT, eN, eE, eg])
    return np.array(rows).reshape(-1, 4)


def main():
    rN, rE, rG = estimate_R()
    r_pos = (rN + rE) / 2
    print("R from stationary ArUco (sample variance):")
    print(f"  sigma_N = {np.sqrt(rN) * 1000:.1f} mm, sigma_E = {np.sqrt(rE) * 1000:.1f} mm, "
          f"sigma_yaw = {np.rad2deg(np.sqrt(rG)):.2f} deg")

    found = [p for p in A1_LOGS if p.exists()]
    if not found:
        raise SystemExit(f"Assignment 1 logs not found in {A1_DIR} (set A1_DIR)")
    E = np.vstack([pair_errors(p) for p in found])
    J, n_sum = len(E), E[:, 0].sum()
    print(f"\nAssignment 1 physical logs: {J} ArUco pairs, "
          f"mean {E[:, 0].mean():.1f} model steps per pair")

    # position: average of the N and E squared errors
    sq_pos = ((E[:, 1] ** 2 + E[:, 2] ** 2) / 2).sum()
    sq_g = (E[:, 3] ** 2).sum()
    q_ne = (sq_pos - 2 * J * r_pos) / n_sum
    q_g = (sq_g - 2 * J * rG) / n_sum
    for name, q in (("Q_NE", q_ne), ("Q_G", q_g)):
        if q <= 0:
            print(f"  warning: {name} <= 0 after subtracting 2R (R too large for this data)")

    print("\nSimple per-step process noise:")
    print(f"  Q_NE = {q_ne:.2e} m^2    (sigma {np.sqrt(max(q_ne, 0)) * 1000:.1f} mm per step)")
    print(f"  Q_G  = {q_g:.2e} rad^2  (sigma {np.rad2deg(np.sqrt(max(q_g, 0))):.2f} deg per step)")
    print("\nCompared with calc_Q.py (values used in laptop.py):")
    print(f"  Q_NE: {q_ne:.2e} vs {Q_NE_FIT:.2e}  (ratio {q_ne / Q_NE_FIT:.1f})")
    print(f"  Q_G : {q_g:.2e} vs {Q_G_FIT:.2e}  (ratio {q_g / Q_G_FIT:.1f})")


if __name__ == "__main__":
    main()
