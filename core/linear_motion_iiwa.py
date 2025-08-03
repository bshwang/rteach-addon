import random
import math
import numpy as np
from scipy.linalg import logm, expm, norm
try:
    import rteach.ext.kuka_iiwa_ik as kuka_iiwa_ik
except ImportError:
    import rteach.ext.kuka_iiwa_ik_py as kuka_iiwa_ik

import rteach.ext.kuka_iiwa_ik_py as kk


def _interpolate_se3(T0, T1, t: float) -> np.ndarray:
    T_rel     = np.linalg.inv(T0) @ T1
    scaled_log = t * logm(T_rel)
    return T0 @ expm(scaled_log)

def _distance_se3(T0, T1) -> float:
    return norm(logm(np.linalg.inv(T0) @ T1), 'fro')

def _interp(a, b, t):  # scalar lerp
    return (1.0 - t) * a + t * b

def _feasible(q_prev, q, ll, ul, max_jump=0.5):
    if np.any(q < ll) or np.any(q > ul):
        return False
    return np.linalg.norm(q_prev - q) < max_jump

import sys

def plan_linear_path(
        q_init: np.ndarray,
        T_goal: np.ndarray,
        props,
        max_try: int = 1000
) -> list[np.ndarray] | None:

    ll = np.radians([-170, -120, -170, -120, -170, -120, -175])
    ul = np.radians([ 170,  120,  170,  120,  170,  120,  175])

    T_start   = kk.kuka_iiwa_fk(q_init)
    pos_start = T_start[:3, 3]
    pos_goal  = T_goal[:3, 3]
    cart_dist = np.linalg.norm(pos_goal - pos_start)

    step_len  = max(props.step_mm * 0.001, 1e-4)
    n_steps   = max(1, round(cart_dist / step_len))

    print(f"[LIN] Total Steps: {n_steps}, Distance (pos only): {round(cart_dist*1000,1)} mm")

    use_fixed_q3 = abs(props.fixed_q3) > 1e-6
    print(f"[LIN] fixed_q3 = {round(math.degrees(props.fixed_q3),2)}° → use_fixed_q3 = {use_fixed_q3}")

    for attempt in range(max_try):
        progress = int(100 * (attempt + 1) / max_try)
        sys.stdout.write(f"\r[LIN] Attempt {attempt+1}/{max_try} (Progress: {progress}%)")
        sys.stdout.flush()

        q_cur   = np.asarray(q_init, float)
        path    = [q_cur]
        r_start = q_init[2]

        if use_fixed_q3:
            r_goal = props.fixed_q3
        else:
            r_goal = random.uniform(ll[2], ul[2])

        ok = True
        for k in range(1, n_steps + 1):
            t = k / n_steps
            T_k = _interpolate_se3(T_start, T_goal, t)
            r_k = _interp(r_start, r_goal, t)
            sols = kuka_iiwa_ik.solve(T_k, th_2=r_k)

            hit = False
            for q in sols:
                q = q.reshape((-1,))
                if np.any(q < ll) or np.any(q > ul):
                    continue
                if np.linalg.norm(q - q_cur) > 0.5:
                    continue
                path.append(q)
                q_cur = q
                hit = True
                break

            if not hit:
                ok = False
                break

        if ok:
            sys.stdout.write("\n") 
            print(f"✅ Solved at try {attempt+1}, path length = {len(path)}")
            return path

    sys.stdout.write("\n") 
    print("❌ Failed to solve Precise LIN path after all attempts")
    return None
