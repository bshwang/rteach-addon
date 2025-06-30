import numpy as np

# Common DH constants for all UR models
alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]

# Model-specific parameters (in meters)
UR_PARAMS = {
    "ur3e":  dict(d1=0.15185, a2=-0.24355, a3=-0.2132,  d4=0.13105, d5=0.08535, d6=0.0921),
    "ur5e":  dict(d1=0.1625,  a2=-0.425,   a3=-0.3922,  d4=0.1333,  d5=0.0997,  d6=0.0996),
    "ur10e": dict(d1=0.1807,  a2=-0.6127,  a3=-0.57155, d4=0.17415, d5=0.11985, d6=0.11655),
    "ur16e": dict(d1=0.1807,  a2=-0.4784,  a3=-0.36,    d4=0.17415, d5=0.11985, d6=0.11655),
    "ur15":  dict(d1=0.1807,  a2=-0.4784,  a3=-0.36,    d4=0.17415, d5=0.11985, d6=0.11655),
    "ur20":  dict(d1=0.245,   a2=-0.795,   a3=-0.302,   d4=0.175,   d5=0.175,   d6=0.1),
    "ur30":  dict(d1=0.245,   a2=-0.795,   a3=-0.302,   d4=0.175,   d5=0.175,   d6=0.1),
}


def dh_transform(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,      ca,      d],
        [0,        0,       0,      1]
    ])


def forward_kinematics(q, params):
    a_vals = [0, params["a2"], params["a3"], 0, 0, 0]
    d_vals = [params["d1"], 0, 0, params["d4"], params["d5"], params["d6"]]
    T = np.eye(4)
    for i in range(6):
        T = T @ dh_transform(a_vals[i], alpha[i], d_vals[i], q[i])
    return T


def inverse_kinematics(T, params):
    d1, a2, a3, d4, d5, d6 = (params[k] for k in ["d1", "a2", "a3", "d4", "d5", "d6"])
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]
    R = T[:3, :3]
    wx = px - d6 * R[0, 2]
    wy = py - d6 * R[1, 2]
    wz = pz - d6 * R[2, 2]

    solutions = []
    
    for s1 in [+1, -1]:
        q1 = np.arctan2(s1 * wy, s1 * wx)
        r = np.hypot(wx, wy)
        x = r
        y = wz - d1

        D = (x**2 + y**2 - a2**2 - a3**2) / (2 * a2 * a3)
        if abs(D) > 1.0:
            continue
        for s3 in [+1, -1]:
            q3 = np.arctan2(s3 * np.sqrt(1 - D**2), D)
            k1 = a2 + a3 * np.cos(q3)
            k2 = a3 * np.sin(q3)
            q2 = np.arctan2(y, x) - np.arctan2(k2, k1)

            a_vals = [0, a2, a3, 0, 0, 0]
            d_vals = [d1, 0, 0, d4, d5, d6]

            T01 = dh_transform(a_vals[0], alpha[0], d_vals[0], q1)
            T12 = dh_transform(a_vals[1], alpha[1], d_vals[1], q2)
            T23 = dh_transform(a_vals[2], alpha[2], d_vals[2], q3)
            T03 = T01 @ T12 @ T23
            R03 = T03[:3, :3]
            R36 = R03.T @ R

            q5 = np.arccos(np.clip(R36[2, 2], -1.0, 1.0))
            for s5 in [+1, -1]:
                if abs(np.sin(q5)) < 1e-6:
                    continue
                q5 = s5 * q5
                q4 = np.arctan2(R36[1, 2] * s5, R36[0, 2] * s5)
                q6 = np.arctan2(R36[2, 1] * s5, -R36[2, 0] * s5)
                q_all = [q1, q2, q3, q4, q5, q6]
                solutions.append(np.array(q_all))
    return solutions


class URSolver:
    def __init__(self, model):
        self.model = model
        self.params = UR_PARAMS[model]

    def forward_kinematics(self, q1, q2, q3, q4, q5, q6):
        return forward_kinematics([q1, q2, q3, q4, q5, q6], self.params)

    def inverse_kinematics(self, T):
        return inverse_kinematics(T, self.params)


ur3e  = URSolver("ur3e")
ur5e  = URSolver("ur5e")
ur10e = URSolver("ur10e")
ur15  = URSolver("ur15")
ur16e = URSolver("ur16e")
ur20  = URSolver("ur20")
ur30  = URSolver("ur30")
