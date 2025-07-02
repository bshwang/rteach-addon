import numpy as np

def calculate_theta1(r13, r23, px, py, d4, d6):
    A1 = px - d6 * r13
    B1 = d6 * r23 - py
    H1 = A1**2 + B1**2 - d4**2
    H2 = np.arctan2(np.sqrt(H1), d4)
    theta1a = np.arctan2(A1, B1) + H2
    theta1b = np.arctan2(A1, B1) - H2
    return theta1a, theta1b

def calculate_theta5(c1, s1, r11, r12, r13, r21, r22, r23):
    c5 = s1 * r13 - c1 * r23
    s5 = np.sqrt((s1 * r11 - c1 * r21)**2 + (s1 * r12 - c1 * r22)**2)
    theta5a = np.arctan2(s5, c5)
    theta5b = np.arctan2(-s5, c5)
    return theta5a, theta5b

def calculate_theta6(sign5, c1, s1, r11, r12, r21, r22):
    H1 = c1 * r22 - s1 * r12
    H2 = s1 * r11 - c1 * r21
    if abs(H1) < 1e-12:
        H1 = 0.0
    if abs(H2) < 1e-12:
        H2 = 0.0
    theta6 = np.arctan2(sign5 * H1, sign5 * H2)
    return theta6

def calculate_theta2(KS, KC, c3, s3, a2, a3):
    return np.arctan2(KS, KC) - np.arctan2(a3 * s3, a3 * c3 + a2)

def fk(q, p):
    a = [0, p["a2"], p["a3"], 0, 0, 0]
    d = [p["d1"], 0, 0, p["d4"], p["d5"], p["d6"]]
    alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]

    def dh(a, alpha, d, theta):
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,      ca,      d],
            [0,        0,       0,      1]
        ])

    T = np.eye(4)
    for i in range(6):
        T = T @ dh(a[i], alpha[i], d[i], q[i])
    return T

def ur_inverse_kinematics(T, d1, d4, d5, d6, a2, a3):
    r11,r12,r13 = T[0,0], T[0,1], T[0,2]
    r21,r22,r23 = T[1,0], T[1,1], T[1,2]
    r31,r32,r33 = T[2,0], T[2,1], T[2,2]
    px,py,pz = T[0,3], T[1,3], T[2,3]

    solutions = np.zeros((8,6))
    t1a, t1b = calculate_theta1(r13, r23, px, py, d4, d6)
    solutions[:4,0] = t1a
    solutions[4:,0] = t1b

    for i in [0, 4]:
        t1 = solutions[i,0]
        c1 = np.cos(t1)
        s1 = np.sin(t1)
        t5a, t5b = calculate_theta5(c1, s1, r11, r12, r13, r21, r22, r23)
        solutions[i+0,4] = t5a
        solutions[i+1,4] = t5b
        solutions[i+2,4] = t5a
        solutions[i+3,4] = t5b
        for j in range(4):
            s5 = np.sin(solutions[i+j,4])
            solutions[i+j,5] = calculate_theta6(s5, c1, s1, r11, r12, r21, r22)

    for i in range(8):
        t1, t5, t6 = solutions[i,0], solutions[i,4], solutions[i,5]
        c1, s1 = np.cos(t1), np.sin(t1)
        c5 = np.cos(t5)
        c6, s6 = np.cos(t6), np.sin(t6)

        px_eff = px - d6 * r13
        py_eff = py - d6 * r23
        pz_eff = pz - d6 * r33

        x = np.hypot(px_eff - c1 * d4, py_eff - s1 * d4)
        y = pz_eff - d1

        D = (x**2 + y**2 - a2**2 - a3**2) / (2 * a2 * a3)
        if abs(D) > 1.0:
            continue

        t3 = np.arccos(D)
        solutions[i,2] = t3
        s3, c3 = np.sin(t3), np.cos(t3)

        k1 = a2 + a3 * c3
        k2 = a3 * s3
        t2 = np.arctan2(y, x) - np.arctan2(k2, k1)
        solutions[i,1] = t2

        # compute T03 and T36 for q4
        def dh(a, alpha, d, theta):
            ca, sa = np.cos(alpha), np.sin(alpha)
            ct, st = np.cos(theta), np.sin(theta)
            return np.array([
                [ct, -st * ca,  st * sa, a * ct],
                [st,  ct * ca, -ct * sa, a * st],
                [0,       sa,      ca,      d],
                [0,        0,       0,      1]
            ])

        T01 = dh(0,     np.pi/2, d1, t1)
        T12 = dh(a2,    0,       0,  t2)
        T23 = dh(a3,    0,       0,  t3)
        T03 = T01 @ T12 @ T23
        R03 = T03[:3,:3]
        R36 = R03.T @ T[:3,:3]

        t4 = np.arctan2(R36[1,2], R36[0,2])
        solutions[i,3] = t4

    return solutions

class URSolver:
    def __init__(self, model_name):
        self.model_name = model_name
        self.params = {
            "ur3e":  dict(d1=0.15185, a2=-0.24355, a3=-0.2132,  d4=0.13105, d5=0.08535, d6=0.0921),
            "ur5e":  dict(d1=0.1625,  a2=-0.425,   a3=-0.3922,  d4=0.1333,  d5=0.0997,  d6=0.0996),
            "ur10e": dict(d1=0.1807,  a2=-0.6127,  a3=-0.57155, d4=0.17415, d5=0.11985, d6=0.11655),
            "ur16e": dict(d1=0.1807,  a2=-0.4784,  a3=-0.36,    d4=0.17415, d5=0.11985, d6=0.11655),
            "ur15":  dict(d1=0.1807,  a2=-0.4784,  a3=-0.36,    d4=0.17415, d5=0.11985, d6=0.11655),
            "ur20":  dict(d1=0.245,   a2=-0.795,   a3=-0.302,   d4=0.175,   d5=0.175,   d6=0.1),
            "ur30":  dict(d1=0.245,   a2=-0.795,   a3=-0.302,   d4=0.175,   d5=0.175,   d6=0.1),
        }[model_name]

    def forward_kinematics(self, *q):
        return fk(q, self.params)

    def inverse_kinematics(self, T):
        p = self.params
        return ur_inverse_kinematics(T, p["d1"], p["d4"], p["d5"], p["d6"], p["a2"], p["a3"])

ur3e  = URSolver("ur3e")
ur5e  = URSolver("ur5e")
ur10e = URSolver("ur10e")
ur15  = URSolver("ur15")
ur16e = URSolver("ur16e")
ur20  = URSolver("ur20")
ur30  = URSolver("ur30")
