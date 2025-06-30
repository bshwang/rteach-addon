"""
Precise Cartesian linear motion planner for KUKA iiwa
· Lie-group (SE3) interpolation
· Per-step IK with joint-limit / continuity check
"""
import random
import numpy as np
from scipy.linalg import logm, expm, norm
try:
    import rteach.ext.kuka_iiwa_ik as kuka_iiwa_ik
except ImportError:
    import rteach.ext.kuka_iiwa_ik_py as kuka_iiwa_ik

import rteach.ext.kuka_iiwa_ik_py as kk


def _interpolate_se3(T0, T1, t: float) -> np.ndarray:
    """Lie-group interpolation between two 4×4 SE3 matrices"""
    T_rel     = np.linalg.inv(T0) @ T1
    scaled_log = t * logm(T_rel)
    return T0 @ expm(scaled_log)

def _distance_se3(T0, T1) -> float:
    """Frobenius-norm distance on SE3"""
    return norm(logm(np.linalg.inv(T0) @ T1), 'fro')

def _interp(a, b, t):  # scalar lerp
    return (1.0 - t) * a + t * b

def _feasible(q_prev, q, ll, ul, max_jump=0.5):
    """joint-limit & continuity"""
    if np.any(q < ll) or np.any(q > ul):
        return False
    return np.linalg.norm(q_prev - q) < max_jump

def plan_linear_path(
        q_init: np.ndarray,
        T_goal: np.ndarray,
        props,
        max_try: int = 1000
) -> list[np.ndarray] | None:
    """
    Returns list of joint vectors (incl. q_init), or None if no path found.
    · props.step_mm  : Cartesian step size (mm)
    · props.fixed_q3 : elbow angle (0 → random search)
    """
    import math
    ll = np.radians([-170, -120, -170, -120, -170, -120, -175])
    ul = np.radians([ 170,  120,  170,  120,  170,  120,  175])

    T_start   = kk.kuka_iiwa_fk(q_init)
    cart_dist = _distance_se3(T_start, T_goal)
    step_len  = max(props.step_mm * 0.001, 1e-4)
    n_steps   = max(1, round(cart_dist / step_len))
    
    use_fixed_q3 = abs(props.fixed_q3) > 1e-6

    print(f"[LIN] fixed_q3 = {round(math.degrees(props.fixed_q3),2)}° → use_fixed_q3 = {use_fixed_q3}")
    print(f"[LIN] Total Steps: {n_steps}, Distance: {round(cart_dist*1000,1)} mm")

    for attempt in range(max_try):
        q_cur   = np.asarray(q_init, float)
        path    = [q_cur]
        r_start = q_init[2]

        if use_fixed_q3:
            r_goal = props.fixed_q3
        else:
            r_goal = random.uniform(ll[2], ul[2])

        if attempt == 0:
            print(f"➤ Try {attempt+1}: elbow = {round(math.degrees(r_goal),1)}°")

        ok = True
        for k in range(1, n_steps + 1):
            T_k = _interpolate_se3(T_start, T_goal, k / n_steps)
            r_k = _interp(r_start, r_goal, k / n_steps)
            sols = kuka_iiwa_ik.solve(T_k, th_2=r_k)

            if attempt == 0 and k == 1:
                pos = T_k[:3,3]*1000
                print(f"  Step {k}: TCP = {np.round(pos,1)} mm, #sols = {len(sols)}")

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
            print(f"Solved at try {attempt+1}, path length = {len(path)}")
            return path

    print("Failed to solve Precise LIN path")
    return None
