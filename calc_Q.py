"""
Estimate the EKF process noise Q from logged physical runs (calc_Q.py).

Idea
----
Restart the motion model at ArUco fix k, propagate it with the logged
*measured* wheel speeds (/true_wheel_speeds -> fwd_kinematics ->
rigid_body_kinematics, exactly the laptop.py predict step) until fix k+m and
compare with that ArUco pose. Every window restarts from ArUco, so it does not
matter how the estimate was produced online (Assignment 1 = kinematics only).

Errors are taken in the body frame of the prediction: along-track e_x and
heading e_g. For windows of different lengths T (m = 1..8 ArUco intervals):

    var(e) = c + a*T^2 + b*T
      c : non-accumulating part (ArUco noise while moving, 2 fixes)
      a : persistent velocity error (wheel radius / slip over a few seconds)
      b : random drift independent every step (random walk)

In laptop.py Q is added every loop step dt = 0.1 s and
    per-step along-track growth = Q_NE + dt^2 * Q_DOTX
    per-step heading growth     = Q_G  + dt^2 * Q_DOTG
so the two parts are assigned without double counting:
    Q_NE   = b_x * dt          Q_G    = b_g * dt
    Q_DOTX = a_x * T_aruco/dt  Q_DOTG = a_g * T_aruco/dt
(the a-part is matched over one ArUco interval T_aruco, since the EKF can only
represent white per-step noise).

Data handling
  * ArUco "flips" (one frame 30-80 deg / ~10 cm off) -> 3-sigma clipping
  * stale ArUco frames (identical pose re-sent) are skipped
  * ArUco timestamps lag the wheel-speed timestamps: a constant time shift is
    estimated (minimum heading-error variance) and applied before the fit

Run (from src/, with the course library on PYTHONPATH, e.g. after active.sh):
    python calc_Q.py
"""
import json
from pathlib import Path

import numpy as np
from scipy.optimize import nnls

from model_uos_imr import ActuatorConfiguration, rigid_body_kinematics
from math_uos_imr import Vector

HERE = Path(__file__).resolve().parent
A1_DIR = Path.home() / "Desktop" / "Assignment 1" / "code and log"
A2_DIR = HERE.parent / "logs"

# physical runs only (the *_test_log.json / Test 6-8 files are Webots runs)
A1_LOGS = [A1_DIR / "Test 1/1433.json", A1_DIR / "Test 2/1437.json",
           A1_DIR / "Test 3/1438.json", A1_DIR / "Test 4/1440.json",
           A1_DIR / "Test 5/1441.json"]
A2_LOGS = [A2_DIR / "practical2.json", A2_DIR / "practical3.json"]
STATIONARY_LOGS = [A2_DIR / f"stationary{i}.json" for i in range(1, 6)]

# same actuator model as laptop.py
ddrive = ActuatorConfiguration(0.081, 0.074)

LOOP_DT = 0.1      # laptop.py runs at 10 Hz -> Q is added every 0.1 s
SUB_DT = 0.02      # integration sub-step [s]
T_MIN, T_MAX = 0.4, 3.0  # usable length of one ArUco interval [s]
MIN_MOTION = 0.01  # window must have >= 1 cm of predicted travel per interval
GLITCH_POS, GLITCH_YAW = 0.05, np.deg2rad(10)  # stationary frame jumps above this are flips
WINDOWS = range(1, 9)  # number of ArUco intervals per window


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
            topic = d["topic_name"]
            t_log = float(d["timestamp"])
            m = d["message"]
            if topic == "/aruco":
                # header stamp = camera capture time; in Assignment 1 the camera
                # clock was one hour ahead, so remove whole hours of offset
                t = m["header"]["stamp"]
                t -= round((t - t_log) / 3600.0) * 3600.0
                p = m["pose"]
                aruco.append([t, p["position"]["x"], p["position"]["y"],
                              quat_yaw(p["orientation"])])
            elif topic == "/true_wheel_speeds":
                # x = right wheel, y = left wheel (robot header clock is not
                # reliable, so use the laptop receive time)
                wheels.append([t_log, m["vector"]["x"], m["vector"]["y"]])
    aruco = np.array(aruco)
    wheels = np.array(wheels)
    aruco = aruco[np.argsort(aruco[:, 0])]
    wheels = wheels[np.argsort(wheels[:, 0])]
    return aruco, wheels


def twist(right, left):
    q = Vector(2)
    q[0] = right
    q[1] = left
    return ddrive.fwd_kinematics(q)


def predict(p0, t0, t1, wheels):
    """Propagate pose p0 from t0 to t1 with zero-order-hold wheel speeds."""
    p = Vector(3)
    p[0], p[1], p[2] = p0
    t = t0
    travelled, turned = 0.0, 0.0
    while t < t1 - 1e-9:
        dt = min(SUB_DT, t1 - t)
        i = np.searchsorted(wheels[:, 0], t, side="right") - 1
        if i < 0:
            u = twist(0.0, 0.0)
        else:
            u = twist(wheels[i, 1], wheels[i, 2])
        p = rigid_body_kinematics(p, u, dt)
        travelled += abs(u[0, 0]) * dt
        turned += abs(u[1, 0]) * dt
        t += dt
    return np.array([p[0, 0], p[1, 0], p[2, 0]]), travelled, turned


def aruco_noise():
    """Frame-to-frame ArUco noise from the stationary logs: var(diff)/2."""
    d_pos, d_g = [], []
    for path in STATIONARY_LOGS:
        if not path.exists():
            continue
        aruco, _ = load_log(path)
        d = np.diff(aruco[:, 1:], axis=0)
        d_pos.extend(d[:, 0])
        d_pos.extend(d[:, 1])
        d_g.extend(wrap(d[:, 2]))
    d_pos, d_g = np.array(d_pos), np.array(d_g)
    d_pos = d_pos[np.abs(d_pos) < GLITCH_POS]
    d_g = d_g[np.abs(d_g) < GLITCH_YAW]
    return np.sqrt(np.var(d_pos, ddof=1) / 2), np.sqrt(np.var(d_g, ddof=1) / 2)


def robust_std(x):
    return 1.4826 * np.median(np.abs(x - np.median(x)))


def clip_mask(x, k=3.0):
    """Iterative k-sigma clipping (first pass uses the robust MAD std)."""
    keep = np.ones(len(x), bool)
    for i in range(20):
        c = np.median(x[keep])
        s = robust_std(x[keep]) if i == 0 else np.std(x[keep], ddof=1)
        new = np.abs(x - c) < k * s
        if np.array_equal(new, keep):
            break
        keep = new
    return keep


def window_errors(path, m, shift):
    """Prediction errors over windows of m ArUco intervals: rows [T, e_x, e_g]."""
    aruco, wheels = load_log(path)
    aruco[:, 0] += shift
    rows = []
    for k in range(len(aruco) - m):
        t0, t1 = aruco[k, 0], aruco[k + m, 0]
        T = t1 - t0
        if not (T_MIN * m <= T <= T_MAX * m) or t0 < wheels[0, 0]:
            continue
        if any(np.all(aruco[j + 1, 1:] == aruco[j, 1:]) for j in range(k, k + m)):
            continue  # stale ArUco frame (same pose re-sent)
        pred, travelled, _ = predict(aruco[k, 1:], t0, t1, wheels)
        if travelled < MIN_MOTION * m:
            continue  # robot not moving
        meas = aruco[k + m, 1:]
        dn, de = meas[0] - pred[0], meas[1] - pred[1]
        g = pred[2]
        rows.append([T, np.cos(g) * dn + np.sin(g) * de, wrap(meas[2] - pred[2])])
    return np.array(rows).reshape(-1, 3)


def clipped_var(x):
    keep = clip_mask(x)
    return np.var(x[keep], ddof=1)


def best_shift(paths):
    """ArUco time shift that minimises the one-interval heading error variance."""
    shifts = np.arange(-0.6, 0.01, 0.05)
    v = [clipped_var(np.vstack([window_errors(p, 1, s) for p in paths])[:, 2]) for s in shifts]
    return shifts[int(np.argmin(v))]


def fit_growth(paths, shift):
    """Fit var(e) = c + a T^2 + b T (non-negative) for e_x and e_g."""
    T, V = [], []
    for m in WINDOWS:
        E = np.vstack([window_errors(p, m, shift) for p in paths])
        T.append(np.mean(E[:, 0]))
        V.append([clipped_var(E[:, 1]), clipped_var(E[:, 2])])
    T, V = np.array(T), np.array(V)
    A = np.c_[np.ones_like(T), T**2, T]
    fits = [nnls(A, V[:, i])[0] for i in range(2)]
    return T, V, fits


def analyse(name, paths):
    shift = best_shift(paths)
    T, V, (fx, fg) = fit_growth(paths, shift)
    t_aruco = T[0]
    print(f"\n=== {name}: ArUco time shift {shift:+.2f} s, one ArUco interval = {t_aruco:.2f} s")
    print("  T [s]      " + " ".join(f"{t:7.2f}" for t in T))
    print("  var e_x    " + " ".join(f"{v:7.1e}" for v in V[:, 0]))
    print("  std e_g deg" + " ".join(f"{np.rad2deg(np.sqrt(v)):7.1f}" for v in V[:, 1]))
    print(f"  along : c {fx[0]:.2e}  a {fx[1]:.2e}  b {fx[2]:.2e}")
    print(f"  head  : c {fg[0]:.2e}  a {fg[1]:.2e}  b {fg[2]:.2e}")
    q = {"NE": fx[2] * LOOP_DT, "G": fg[2] * LOOP_DT,
         "DOTX": fx[1] * t_aruco / LOOP_DT, "DOTG": fg[1] * t_aruco / LOOP_DT}
    print(f"  Q_NE   = {q['NE']:.2e} m^2      (sigma {np.sqrt(q['NE']) * 1000:.1f} mm per step)")
    print(f"  Q_G    = {q['G']:.2e} rad^2    (sigma {np.rad2deg(np.sqrt(q['G'])):.2f} deg per step)")
    print(f"  Q_DOTX = {q['DOTX']:.2e} (m/s)^2  (sigma {np.sqrt(q['DOTX']):.4f} m/s)")
    print(f"  Q_DOTG = {q['DOTG']:.2e} (rad/s)^2 (sigma {np.rad2deg(np.sqrt(q['DOTG'])):.2f} deg/s)")
    print(f"  moving ArUco noise per fix: position {np.sqrt(fx[0] / 2) * 1000:.1f} mm, "
          f"heading {np.rad2deg(np.sqrt(fg[0] / 2)):.2f} deg")
    return q


def main():
    s_pos, s_g = aruco_noise()
    print(f"Stationary ArUco frame-to-frame noise: position {s_pos * 1000:.1f} mm, "
          f"heading {np.rad2deg(s_g):.2f} deg")
    q = analyse("Assignment 1 physical (kinematics only) -> used for laptop.py", A1_LOGS)
    analyse("Assignment 2 practical2/3 (comparison; contains the reversing bug)", A2_LOGS)
    print("\nValues for laptop.py (from Assignment 1):")
    print(f"Q_NE_0   = {q['NE']:.2e}")
    print(f"Q_G_0    = {q['G']:.2e}")
    print(f"Q_DOTX_0 = {q['DOTX']:.2e}")
    print(f"Q_DOTG_0 = {q['DOTG']:.2e}")


if __name__ == "__main__":
    main()
