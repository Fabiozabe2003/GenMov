from functions.symbolic_mathematical_functions import *
# --- Floating base transformation ---
def sym_floating_base(q):
    I_R_B = sTrotz(q[5]) @ sTroty(q[4]) @ sTrotx(q[3])
    I_P_B = sTrasl(q[0], q[1], q[2])
    return I_P_B @ I_R_B

# --- Forward kinematics ---
def sym_fkine_left_leg(q, qf, t):
    T_I_B = sym_floating_base(qf)
    T = sTrasl(0, 0.05, -0.085) @ sTrotx(-(ca.pi/2 + ca.pi/4))
    T = T @ sTrotz(q[0])
    T = T @ sTrotx(ca.pi/4) @ sTroty(ca.pi/2) @ sTrotz(q[1])
    T = T @ sTroty(-ca.pi/2) @ sTrotz(q[2])
    T = T @ sTrasl(0, 0.1, 0) @ sTrotz(q[3])
    T = T @ sTrasl(0, 0.1029, 0) @ sTrotz(q[4])
    T = T @ sTrotx(q[5])
    return T_I_B @ T if t in ['c', 'C'] else T

def sym_fkine_left_knee(q, qf, t):
    T_I_B = sym_floating_base(qf)
    T = sTrasl(0, 0.05, -0.085) @ sTrotx(-(ca.pi/2 + ca.pi/4))
    T = T @ sTrotz(q[0])
    T = T @ sTrotx(ca.pi/4) @ sTroty(ca.pi/2) @ sTrotz(q[1])
    T = T @ sTroty(-ca.pi/2) @ sTrotz(q[2])
    T = T @ sTrasl(0, 0.1, 0) @ sTrotz(q[3])
    return T_I_B @ T if t in ['c', 'C'] else T

def sym_fkine_right_leg(q, qf, t):
    T_I_B = sym_floating_base(qf)
    T = sTrasl(0, -0.05, -0.085) @ sTrotx(-ca.pi/4)
    T = T @ sTrotz(q[0])
    T = T @ sTrotx(ca.pi/4) @ sTrotx(q[1])
    T = T @ sTroty(q[2])
    T = T @ sTrasl(0, 0, -0.1) @ sTroty(q[3])
    T = T @ sTrasl(0, 0, -0.1029) @ sTroty(q[4])
    T = T @ sTrotx(q[5])
    return T_I_B @ T if t in ['c', 'C'] else T

def sym_fkine_right_knee(q, qf, t):
    T_I_B = sym_floating_base(qf)
    T = sTrasl(0, -0.05, -0.085) @ sTrotx(-ca.pi/4)
    T = T @ sTrotz(q[0])
    T = T @ sTrotx(ca.pi/4) @ sTrotx(q[1])
    T = T @ sTroty(q[2])
    T = T @ sTrasl(0, 0, -0.1) @ sTroty(q[3])
    return T_I_B @ T if t in ['c', 'C'] else T

def sym_fkine_left_foot_contacts(q, qf, t):
    T0 = sym_fkine_left_leg(q, qf, t)
    return [T0 @ sTrasl(*p) for p in [(-0.03, 0.04519, -0.02), (-0.03, 0.04519, 0.03), (0.08, 0.04519, 0.03), (0.08, 0.04519, -0.02)]]

def sym_fkine_right_foot_contacts(q, qf, t):
    T0 = sym_fkine_right_leg(q, qf, t)
    R = sTrotx(-ca.pi/2)
    return [T0 @ R @ sTrasl(*p) for p in [(-0.03, 0.04519, -0.02), (-0.03, 0.04519, 0.03), (0.08, 0.04519, 0.03), (0.08, 0.04519, -0.02)]]

def sym_fkine_left_foot_constant(q, qf, t):
    return sym_fkine_left_leg(q, qf, t) @ sTrasl(0.03, 0.04519, 0)

def sym_fkine_right_foot_constant(q, qf, t):
    return sym_fkine_right_leg(q, qf, t) @ sTrotx(-ca.pi/2) @ sTrasl(0.03, 0.04519, 0)
