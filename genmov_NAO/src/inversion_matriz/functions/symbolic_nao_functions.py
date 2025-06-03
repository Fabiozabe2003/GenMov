import casadi as ca
from functions.symbolic_lower_body_kinematics import *
from functions.symbolic_upper_body_kinematics import *
cos = ca.cos
sin = ca.sin
pi = ca.pi

# q for functions
q_sym = ca.MX.sym("q", 6)
qf_sym = ca.MX.sym("qf", 6)

# Left arm jacobian
def sym_jacobian_left_arm(q,qf):
    T=sym_fkine_left_arm(q,qf,'b')
    p=T[0:3,3]
    J=ca.jacobian(p,q)
    return J
def sym_fb_jacobian_left_arm(q,qf):
    J1=ca.MX_eye(3)
    J2=-sAntisimetrica(sym_fkine_left_arm(q,qf,'c')[0:3,3]-sym_floating_base(qf)[0:3,3])
    J3=sym_floating_base(qf)[0:3,0:3]@sym_jacobian_left_arm(q,qf)
    J=ca.horzcat(ca.MX.zeros(3,14),J3,ca.MX.zeros(3,6),J1,J2)
    return J
J_fb_left_arm = sym_fb_jacobian_left_arm(q_sym, qf_sym)
sym_J_fb_left_arm = ca.Function("J_fbla", [q_sym, qf_sym], [J_fb_left_arm])

# Left elbow jacobian
def sym_jacobian_left_elbow(q,qf):
    T=sym_fkine_left_elbow(q,qf,'b')
    p=T[0:3,3]
    J=ca.jacobian(p,q)
    return J
def sym_fb_jacobian_left_elbow(q,qf):
    J1=ca.MX_eye(3)
    J2=-sAntisimetrica(sym_fkine_left_elbow(q,qf,'c')[0:3,3]-sym_floating_base(qf)[0:3,3])
    J3=sym_floating_base(qf)[0:3,0:3]@sym_jacobian_left_elbow(q,qf)
    J=ca.horzcat(ca.MX.zeros(3,14),J3,ca.MX.zeros(3,9),J1,J2)
    return J
J_fb_left_elbow = sym_fb_jacobian_left_elbow(q_sym, qf_sym)
sym_J_fb_left_elbow = ca.Function("J_fble", [q_sym, qf_sym], [J_fb_left_elbow])

# Right arm jacobian
def sym_jacobian_right_arm(q,qf):
    T=sym_fkine_right_arm(q,qf,'b')
    p=T[0:3,3]
    J=ca.jacobian(p,q)
    return J
def sym_fb_jacobian_right_arm(q,qf):
    J1=ca.MX_eye(3)
    J2=-sAntisimetrica(sym_fkine_right_arm(q,qf,'c')[0:3,3]-sym_floating_base(qf)[0:3,3])
    J3=sym_floating_base(qf)[0:3,0:3]@sym_jacobian_right_arm(q,qf)
    J=ca.horzcat(ca.MX.zeros(3,20),J3,J1,J2)
    return J
J_fb_right_arm = sym_fb_jacobian_right_arm(q_sym, qf_sym)
sym_J_fb_right_arm = ca.Function("J_fbra", [q_sym, qf_sym], [J_fb_right_arm])

# Right elbow jacobian
def sym_jacobian_right_elbow(q,qf):
    T=sym_fkine_right_elbow(q,qf,'b')
    p=T[0:3,3]
    J=ca.jacobian(p,q)
    return J
def sym_fb_jacobian_right_elbow(q,qf):
    J1=ca.MX_eye(3)
    J2=-sAntisimetrica(sym_fkine_right_elbow(q,qf,'c')[0:3,3]-sym_floating_base(qf)[0:3,3])
    J3=sym_floating_base(qf)[0:3,0:3]@sym_jacobian_right_elbow(q,qf)
    J=ca.horzcat(ca.MX.zeros(3,20),J3,ca.MX.zeros(3,3),J1,J2)
    return J
J_fb_right_elbow = sym_fb_jacobian_right_elbow(q_sym, qf_sym)
sym_J_fb_right_elbow = ca.Function("J_fbre", [q_sym, qf_sym], [J_fb_right_elbow])

# Left leg jacobian
def sym_jacobian_left_leg(q,qf):
    T=sym_fkine_left_leg(q,qf,'b')
    p=T[0:3,3]
    J=ca.jacobian(p,q)
    return J
def sym_fb_jacobian_left_leg(q,qf):
    J1=ca.MX_eye(3)
    J2=-sAntisimetrica(sym_fkine_left_leg(q,qf,'c')[0:3,3]-sym_floating_base(qf)[0:3,3])
    J3=sym_floating_base(qf)[0:3,0:3]@sym_jacobian_left_leg(q,qf)
    J=ca.horzcat(ca.MX.zeros(3,2),J3,ca.MX.zeros(3,18),J1,J2)
    return J
J_fb_left_leg = sym_fb_jacobian_left_leg(q_sym, qf_sym)
sym_J_fb_left_leg = ca.Function("J_fbll", [q_sym, qf_sym], [J_fb_left_leg])

# Right leg jacobian
def sym_jacobian_right_leg(q,qf):
    T=sym_fkine_right_leg(q,qf,'b')
    p=T[0:3,3]
    J=ca.jacobian(p,q)
    return J
def sym_fb_jacobian_right_leg(q,qf):
    J1=ca.MX_eye(3)
    J2=-sAntisimetrica(sym_fkine_right_leg(q,qf,'c')[0:3,3]-sym_floating_base(qf)[0:3,3])
    J3=sym_floating_base(qf)[0:3,0:3]@sym_jacobian_right_leg(q,qf)
    J=ca.horzcat(ca.MX.zeros(3,8),J3,ca.MX.zeros(3,12),J1,J2)
    return J
J_fb_right_leg = sym_fb_jacobian_right_leg(q_sym, qf_sym)
sym_J_fb_right_leg = ca.Function("J_fbrl", [q_sym, qf_sym], [J_fb_right_leg])

# Left knee jacobian
def sym_jacobian_left_knee(q,qf):
    T=sym_fkine_left_knee(q,qf,'b')
    p=T[0:3,3]
    J=ca.jacobian(p,q)
    return J
def sym_fb_jacobian_left_knee(q,qf):
    J1=ca.MX_eye(3)
    J2=-sAntisimetrica(sym_fkine_left_knee(q,qf,'c')[0:3,3]-sym_floating_base(qf)[0:3,3])
    J3=sym_floating_base(qf)[0:3,0:3]@sym_jacobian_left_knee(q,qf)
    J=ca.horzcat(ca.MX.zeros(3,2),J3,ca.MX.zeros(3,20),J1,J2)
    return J
J_fb_left_knee = sym_fb_jacobian_left_knee(q_sym, qf_sym)
sym_J_fb_left_knee = ca.Function("J_fblk", [q_sym, qf_sym], [J_fb_left_knee])

# Right knee jacobian
def sym_jacobian_right_knee(q,qf):
    T=sym_fkine_right_knee(q,qf,'b')
    p=T[0:3,3]
    J=ca.jacobian(p,q)
    return J
def sym_fb_jacobian_right_knee(q,qf):
    J1=ca.MX_eye(3)
    J2=-sAntisimetrica(sym_fkine_right_knee(q,qf,'c')[0:3,3]-sym_floating_base(qf)[0:3,3])
    J3=sym_floating_base(qf)[0:3,0:3]@sym_jacobian_right_knee(q,qf)
    J=ca.horzcat(ca.MX.zeros(3,8),J3,ca.MX.zeros(3,14),J1,J2)
    return J
J_fb_right_knee = sym_fb_jacobian_right_knee(q_sym, qf_sym)
sym_J_fb_right_knee = ca.Function("J_fbrk", [q_sym, qf_sym], [J_fb_right_knee])

# Left foot contacts jacobian
def sym_jacobian_left_foot_contacts(q,qf):
    Ts=sym_fkine_left_foot_contacts(q,qf,'b')
    Js=[]
    for T in Ts:
        p=T[0:3,3]
        J=ca.jacobian(p,q)
        Js.append(J)
    return Js #evaluar si Js para seleccionar
def sym_fb_jacobian_left_foot_contacts(q,qf):
    Ts=sym_fkine_left_foot_contacts(q,qf,'c')
    Js=sym_jacobian_left_foot_contacts(q,qf)
    Tfb=sym_floating_base(qf)
    J_list=[]
    for i in range(4):
        dP=Ts[i][0:3,3]-Tfb[0:3,3]
        J1=ca.MX_eye(3)
        J2=-sAntisimetrica(dP)
        J3=Tfb[0:3,0:3]@Js[i]
        J_row=ca.horzcat(ca.MX.zeros(3,2),J3,ca.MX.zeros(3,18),J1,J2)
        J_list.append(J_row)
    J_full=ca.vertcat(*J_list)
    return J_full
J_fb_left_foot_contacts = sym_fb_jacobian_left_foot_contacts(q_sym, qf_sym)
sym_J_fb_left_foot_contacts = ca.Function("J_fblfc", [q_sym, qf_sym], [J_fb_left_foot_contacts])

# Right foot contacts jacobian
def sym_jacobian_right_foot_contacts(q,qf):
    Ts=sym_fkine_right_foot_contacts(q,qf,'b')
    Js=[]
    for T in Ts:
        p=T[0:3,3]
        J=ca.jacobian(p,q)
        Js.append(J)
    return Js #evaluar si Js para seleccionar
def sym_fb_jacobian_right_foot_contacts(q,qf):
    Ts=sym_fkine_right_foot_contacts(q,qf,'c')
    Js=sym_jacobian_right_foot_contacts(q,qf)
    Tfb=sym_floating_base(qf)
    J_list=[]
    for i in range(4):
        dP=Ts[i][0:3,3]-Tfb[0:3,3]
        J1=ca.MX_eye(3)
        J2=-sAntisimetrica(dP)
        J3=Tfb[0:3,0:3]@Js[i]
        J_row=ca.horzcat(ca.MX.zeros(3,2),J3,ca.MX.zeros(3,18),J1,J2)
        J_list.append(J_row)
    J_full=ca.vertcat(*J_list)
    return J_full
J_fb_right_foot_contacts = sym_fb_jacobian_right_foot_contacts(q_sym, qf_sym)
sym_J_fb_right_foot_contacts = ca.Function("J_fbrfc", [q_sym, qf_sym], [J_fb_right_foot_contacts])

# Contacts jacobian - FZ
def build_Jacobian_contacts():
    q_i_sym = ca.MX.sym('q_i', 32)

    q_i_lleg = q_i_sym[2:8]
    q_i_rleg = q_i_sym[8:14]
    q_i_fb = q_i_sym[26:32]

    T_l1, T_l2, T_l3, T_l4 = sym_fkine_left_foot_contacts( q_i_lleg,q_i_fb,"c" )
    T_r1, T_r2, T_r3, T_r4 = sym_fkine_right_foot_contacts( q_i_rleg,q_i_fb,"c")

    # Extraer posiciones de cada transformación
    positions = [T[0:3, 3] for T in [T_l1, T_l2, T_l3, T_l4, T_r1, T_r2, T_r3, T_r4]]
    positions = ca.vertcat(*positions)
    J_i_func = ca.Function('J_i_func', [q_i_sym], [ca.jacobian(positions, q_i_sym)])

    return J_i_func


# Left foot constant jacobian
def sym_jacobian_left_foot_constant(q,qf):
    T=sym_fkine_left_foot_constant(q,qf,'b')
    p = T[0:3, 3]
    Jp = ca.jacobian(p, q)  # 3xN
    R = T[0:3, 0:3]
    Jr_cols=[]
    dR = ca.jacobian(R.reshape((9,1)), q)
    for i in range(q.shape[0]):
        dRi_mat=dR[:,i].reshape((3,3))
        omega_i_skew=dRi_mat @ R.T
        omega_i = ca.vertcat(
            omega_i_skew[2, 1] - omega_i_skew[1, 2],
            omega_i_skew[0, 2] - omega_i_skew[2, 0],
            omega_i_skew[1, 0] - omega_i_skew[0, 1]
        ) / 2.0
        Jr_cols.append(omega_i)
    Jr=ca.horzcat(*Jr_cols)
    J=ca.vertcat(Jp, Jr)
    return J
def sym_fb_jacobian_left_foot_constant(q,qf):
    J_link = sym_jacobian_left_foot_constant(q, qf)  # 6x6
    T=sym_fkine_left_foot_constant(q,qf,'c')
    R_base = sym_floating_base(qf)[0:3, 0:3]
    p_base = sym_floating_base(qf)[0:3, 3]
    p_foot = T[0:3, 3]
    roll=qf[3];pitch=qf[4];yaw=qf[5]
    E0=ca.vertcat(
        ca.horzcat(1,0,-ca.sin(pitch)),
        ca.horzcat(0,ca.cos(roll),ca.sin(roll)*ca.cos(pitch)),
        ca.horzcat(0,-ca.sin(roll),ca.cos(roll)*ca.cos(pitch))
    )
    J1 = ca.MX_eye(3)
    J2 = -sAntisimetrica(p_foot - p_base)@E0
    J3 = R_base@J_link[0:3,:]
    Jt = ca.horzcat(ca.MX.zeros(3,2),J3,ca.MX.zeros(3,18),J1,J2)
    J1 = ca.MX.zeros(3,3)
    
    J2 = E0
    J3 = R_base @ J_link[3:6,:]
    Jr = ca.horzcat(ca.MX.zeros(3,2),J3,ca.MX.zeros(3,18),J1,J2)
    J = ca.vertcat(Jt,Jr)
    return J
J_fb_left_foot_constant = sym_fb_jacobian_left_foot_constant(q_sym, qf_sym)
sym_J_fb_left_foot_constant = ca.Function("J_fblfcte", [q_sym, qf_sym], [J_fb_left_foot_constant])

# Right foot constant jacobian
def sym_jacobian_right_foot_constant(q,qf):
    T=sym_fkine_right_foot_constant(q,qf,'b')
    p = T[0:3, 3]
    Jp = ca.jacobian(p, q)  # 3xN
    R = T[0:3, 0:3]
    Jr_cols=[]
    dR = ca.jacobian(R.reshape((9,1)), q)
    for i in range(q.shape[0]):
        dRi_mat=dR[:,i].reshape((3,3))
        omega_i_skew=dRi_mat @ R.T
        omega_i = ca.vertcat(
            omega_i_skew[2, 1] - omega_i_skew[1, 2],
            omega_i_skew[0, 2] - omega_i_skew[2, 0],
            omega_i_skew[1, 0] - omega_i_skew[0, 1]
        ) / 2.0
        Jr_cols.append(omega_i)
    Jr=ca.horzcat(*Jr_cols)
    J=ca.vertcat(Jp, Jr)
    return J
def sym_fb_jacobian_right_foot_constant(q,qf):
    J_link = sym_jacobian_right_foot_constant(q, qf)  # 6x6
    T=sym_fkine_right_foot_constant(q,qf,'c')
    R_base = sym_floating_base(qf)[0:3, 0:3]
    p_base = sym_floating_base(qf)[0:3, 3]
    p_foot = T[0:3, 3]
    roll=qf[3];pitch=qf[4];yaw=qf[5]
    E0=ca.vertcat(
        ca.horzcat(1,0,-ca.sin(pitch)),
        ca.horzcat(0,ca.cos(roll),ca.sin(roll)*ca.cos(pitch)),
        ca.horzcat(0,-ca.sin(roll),ca.cos(roll)*ca.cos(pitch))
    )
    J1 = ca.MX_eye(3)
    J2 = -sAntisimetrica(p_foot - p_base)@E0
    J3 = R_base@J_link[0:3,:]
    Jt = ca.horzcat(ca.MX.zeros(3,2),J3,ca.MX.zeros(3,18),J1,J2)
    J1 = ca.MX.zeros(3,3)
    J2 = E0
    J3 = R_base @ J_link[3:6,:]
    Jr = ca.horzcat(ca.MX.zeros(3,2),J3,ca.MX.zeros(3,18),J1,J2)
    J = ca.vertcat(Jt,Jr)
    return J
J_fb_right_foot_constant = sym_fb_jacobian_right_foot_constant(q_sym, qf_sym)
sym_J_fb_right_foot_constant = ca.Function("J_fbrfcte", [q_sym, qf_sym], [J_fb_right_foot_constant])

#----Jacobiano
q_temp = ca.MX.sym("q_temp", 32)
dq_temp = ca.MX.sym("dq_temp", 32,1)
# 2. Extract slices
q_lleg_temp = q_temp[2:8]
q_rleg_temp = q_temp[8:14]
q_fb_temp   = q_temp[26:32]
# 3. Build symbolic J_point
J_point_temp = ca.vertcat(
    sym_J_fb_left_foot_constant(q_lleg_temp, q_fb_temp),
    sym_J_fb_right_foot_constant(q_rleg_temp, q_fb_temp)
)
# 4. Compute symbolic directional derivative
dJdq = ca.jtimes(J_point_temp, q_temp, dq_temp)
# 5. Wrap in CasADi function
dJdq = ca.Function("dJdq", [q_temp, dq_temp], [dJdq])




#J_eval = sym_J_fb_right_arm(q_sym, qf_sym)  # Now it's compatible
J=sym_J_fb_left_foot_contacts(q_sym,qf_sym)
#print(J.shape)