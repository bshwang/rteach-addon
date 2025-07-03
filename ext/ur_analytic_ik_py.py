# rteach/ext/ur_analytic_ik_py.py
import numpy as np

ALPHA = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]

UR_PARAMS = {
    "ur3e":  dict(d1=0.15185, a2=-0.24355, a3=-0.2132,  d4=0.13105, d5=0.08535,  d6=0.0921),
    "ur5e":  dict(d1=0.1625,  a2=-0.425,   a3=-0.3922,  d4=0.1333,  d5=0.0997,   d6=0.0996),
    "ur10e": dict(d1=0.1807,  a2=-0.6127,  a3=-0.57155, d4=0.17415, d5=0.11985, d6=0.11655),
    "ur15":  dict(d1=0.1807,  a2=-0.4784,  a3=-0.36,    d4=0.17415, d5=0.11985, d6=0.11655),
    "ur16e": dict(d1=0.1807,  a2=-0.4784,  a3=-0.36,    d4=0.17415, d5=0.11985, d6=0.11655),
    "ur20":  dict(d1=0.245,   a2=-0.795,   a3=-0.302,   d4=0.175,   d5=0.175,   d6=0.1),
    "ur30":  dict(d1=0.245,   a2=-0.795,   a3=-0.302,   d4=0.175,   d5=0.175,   d6=0.1),
}

def dh(a, alpha, d, th):
    ca,sa=np.cos(alpha),np.sin(alpha)
    ct,st=np.cos(th),np.sin(th)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0. ,    sa,     ca,   d ],
                     [0. ,  0.0,    0.0,  1. ]])

def wrap(q): return (q+np.pi)%(2*np.pi)-np.pi

def fk_full(q, p):
    a = [0.0, p["a2"], p["a3"], 0.0, 0.0, 0.0]
    d = [p["d1"], 0.0,     0.0,     p["d4"], p["d5"], p["d6"]]
    T = np.eye(4)
    for i in range(6):
        T = T @ dh(a[i], ALPHA[i], d[i], q[i])
    return T

def ik_solve(T, p):
    d1,a2,a3 = p["d1"],p["a2"],p["a3"]
    d4,d5,d6 = p["d4"],p["d5"],p["d6"]
    R=T[:3,:3]; px,py,pz=T[0,3],T[1,3],T[2,3]
    print(f"[IK DEBUG] px,py,pz = {px:.4f},{py:.4f},{pz:.4f}")

    A1 = px - d6*R[0,2]
    B1 = d6*R[1,2] - py                      # sign exactly per C++
    r = np.hypot(A1,B1)
    if r <= d4: 
        print("  → inside d4 radius"); return []
    k = np.sqrt(r*r - d4*d4)
    t1 = [wrap(np.arctan2(A1,B1)+np.arctan2(k, d4)),
          wrap(np.arctan2(A1,B1)+np.arctan2(-k,d4))]
    print(f"[IK DEBUG] theta1 candidates = {[np.degrees(t) for t in t1]}")

    sols=[]
    for i,t1i in enumerate(t1):
        c1,s1=np.cos(t1i),np.sin(t1i)

        # eq.29–32  → theta5, theta6, theta234
        c5 = (s1*R[0,2]-c1*R[1,2])
        s5 = np.hypot(s1*R[0,0]-c1*R[1,0], s1*R[0,1]-c1*R[1,1])
        if s5<1e-12: s5=1e-12
        for t5 in (wrap(np.arctan2( s5,c5)), wrap(np.arctan2(-s5,c5))):
            sign5=np.sign(np.sin(t5)) or 1.0
            t6 = wrap(np.arctan2(sign5*(c1*R[1,1]-s1*R[0,1]),
                                 sign5*(s1*R[0,0]-c1*R[1,0])))
            c5v,s5v=np.cos(t5),np.sin(t5)
            A234 = c1*R[0,0]+s1*R[1,0]
            H1   = c5v*np.cos(t6)*R[2,0]-np.sin(t6)*A234
            H2   = c5v*np.cos(t6)*A234+np.sin(t6)*R[2,0]
            t234 = np.arctan2(H1,H2)
            c234,s234=np.cos(t234),np.sin(t234)

            KC = (c1*px)+(s1*py) - s234*d5 + c234*s5v*d6
            KS =  pz - d1        + c234*d5 + s234*s5v*d6
            # eq.52,55 → theta3
            d_val = (KC*KC+KS*KS - a2*a2 - a3*a3)/(2*a2*a3)
            if abs(d_val)>1.0+1e-9: continue
            for t3 in (np.arccos(np.clip(d_val,-1,1)),
                       -np.arccos(np.clip(d_val,-1,1))):
                c3,s3=np.cos(t3),np.sin(t3)
                t2 = wrap(np.arctan2(KS,KC)-np.arctan2(a3*s3,a2+a3*c3))
                t4 = wrap(t234 - t2 - t3)
                sols.append(wrap(np.array([t1i,t2,t3,t4,t5,t6])))

    uniq=[]
    for q in sols:
        if not any(np.linalg.norm(q-u)<1e-6 for u in uniq): uniq.append(q)
    print(f"[IK DEBUG] unique count = {len(uniq)}")

    final=[]
    for q in uniq:
        T_fk = np.eye(4)
        a=[0,a2,a3,0,0,0]; d=[d1,0,0,d4,d5,d6]
        for k,qi in enumerate(q): T_fk=T_fk@dh(a[k],ALPHA[k],d[k],qi)
        pe=np.linalg.norm(T_fk[:3,3]-T[:3,3])
        re=np.linalg.norm(T_fk[:3,:3]-T[:3,:3])
        print(f"[IK DEBUG] err pos={pe:.6f}, rot={re:.6f}")
        if pe<1e-3 and re<1e-2: final.append(q)
    print(f"[IK DEBUG] final count = {len(final)}")
    return final

class URSolver:
    def __init__(self, model): self.p=UR_PARAMS[model]
    def forward_kinematics(self,*q): return fk_full(q,self.p)
    def inverse_kinematics(self,T):
        print("=== UR inverse start ===")
        r=ik_solve(np.asarray(T,float),self.p)
        print("=== UR inverse end ===")
        return r

ur3e  = URSolver("ur3e")
ur5e  = URSolver("ur5e")
ur10e = URSolver("ur10e")
ur15  = URSolver("ur15")
ur16e = URSolver("ur16e")
ur20  = URSolver("ur20")
ur30  = URSolver("ur30")
