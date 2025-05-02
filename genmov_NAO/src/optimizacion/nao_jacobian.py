import casadi as ca

cos=ca.cos; sin=ca.sin; pi=ca.pi

# Funciones de Rotación
def sTrotx(ang):
    Tx = ca.vertcat(
        ca.horzcat(1, 0, 0, 0),
        ca.horzcat(0, ca.cos(ang), -ca.sin(ang), 0),
        ca.horzcat(0, ca.sin(ang),  ca.cos(ang), 0),
        ca.horzcat(0, 0, 0, 1)
    )
    return Tx

def sTroty(ang):
    Ty = ca.vertcat(
        ca.horzcat( ca.cos(ang), 0, ca.sin(ang), 0),
        ca.horzcat( 0, 1, 0, 0),
        ca.horzcat(-ca.sin(ang), 0, ca.cos(ang), 0),
        ca.horzcat( 0, 0, 0, 1)
    )
    return Ty

def sTrotz(ang):
    Tz = ca.vertcat(
        ca.horzcat(ca.cos(ang), -ca.sin(ang), 0, 0),
        ca.horzcat(ca.sin(ang),  ca.cos(ang), 0, 0),
        ca.horzcat(0, 0, 1, 0),
        ca.horzcat(0, 0, 0, 1)
    )
    return Tz

def sTrasl(x, y, z):
    T = ca.vertcat(
        ca.horzcat(1, 0, 0, x),
        ca.horzcat(0, 1, 0, y),
        ca.horzcat(0, 0, 1, z),
        ca.horzcat(0, 0, 0, 1)
    )
    return T


def sfloating_base(q):
    I_R_B=sTrotz(q[5])@sTroty(q[4])@sTrotx(q[3])
    I_P_B=sTrasl(q[0],q[1],q[2])
    T_I_B=I_P_B@I_R_B
    return T_I_B

def sfkine_left_leg(q,qf,t):
    T_I_B=sfloating_base(qf)
    T0=sTrasl(0, 0.05, -0.085)@sTrotx(-(pi/2+pi/4))   #Torso to LHipYawPitch
    T1=sTrotz(q[0]) #z0
    T2=sTrotx(pi/4)@sTroty(pi/2)@sTrotz(q[1]) #z1
    T3=sTroty(-pi/2)@sTrotz(q[2]) #z2
    T4=sTrasl(0,0.1,0)@sTrotz(q[3])
    T5=sTrasl(0,0.1029,0)@sTrotz(q[4])
    T6=sTrotx(q[5])
    if t=='c' or t=='C':
        T=T_I_B@T0@T1@T2@T3@T4@T5@T6
    else: 
        T=T0@T1@T2@T3@T4@T5@T6
    return T

def sfkine_right_leg(q,qf,t):
    T_I_B=sfloating_base(qf)
    T0=sTrasl(0, -0.05, -0.085)@sTrotx(-(pi/4))   #Torso to RHipYawPitch
    T1=sTrotz(q[0]) #z0
    T2=sTrotx(pi/4)@sTrotx(q[1]) #z1
    T3=sTroty(q[2]) #z2
    T4=sTrasl(0,0,-0.1)@sTroty(q[3])
    T5=sTrasl(0,0,-0.1029)@sTroty(q[4])
    T6=sTrotx(q[5])
    if t=='c' or t=='C':
        T=T_I_B@T0@T1@T2@T3@T4@T5@T6
    else: 
        T=T0@T1@T2@T3@T4@T5@T6
    return T


def sfkine_left_foot_contacts(q,qf,t):
    T0=sfkine_left_leg(q,qf,t)
    T1=sTrasl(-0.03,0.04519,-0.02)
    T2=sTrasl(-0.03,0.04519,0.03)
    T3=sTrasl(0.08,0.04519,0.03)
    T4=sTrasl(0.08,0.04519,-0.02)
    return T0@T1,T0@T2,T0@T3,T0@T4

def sfkine_right_foot_contacts(q,qf,t):
    T0=sfkine_right_leg(q,qf,t)
    T1=sTrasl(-0.03,0.04519,-0.02)
    T2=sTrasl(-0.03,0.04519,0.03)
    T3=sTrasl(0.08,0.04519,0.03)
    T4=sTrasl(0.08,0.04519,-0.02)
    R=sTrotx(-pi/2)
    return T0@R@T1,T0@R@T2,T0@R@T3,T0@R@T4



joint_indices = {
    "l_arm": slice(14 + 6, 20 + 6),
    "r_arm": slice(20 + 6 , 26 + 6),
    "f_base": slice(0,6),
    "l_leg": slice(2  + 6, 8 + 6),
    "r_leg": slice(8 + 6, 14 +6),
}



def build_Jacobian_contacts():
    q_i_sym = ca.MX.sym('q_i', 32)

    q_i_lleg = q_i_sym[joint_indices["l_leg"]]
    q_i_rleg = q_i_sym[joint_indices["r_leg"]]
    q_i_base = q_i_sym[joint_indices["f_base"]]


    T_l1, T_l2, T_l3, T_l4 = sfkine_left_foot_contacts( q_i_lleg,q_i_base,"c" )
    T_r1, T_r2, T_r3, T_r4 = sfkine_right_foot_contacts( q_i_rleg,q_i_base,"c")

    # Extraer posiciones de cada transformación
    positions = [T[0:3, 3] for T in [T_l1, T_l2, T_l3, T_l4, T_r1, T_r2, T_r3, T_r4]]

    positions = ca.vertcat(*positions)

    J_i_func = ca.Function('J_i_func', [q_i_sym], [ca.jacobian(positions, q_i_sym)])

    return J_i_func



# N = 25
# Dof = 32  
# Dof_act= 26
# i = 0

# q = ca.MX.sym('q',Dof*N) # variable 'q' de cuero completo
# q_i = q[Dof*i:Dof*(i+1)]
# tau = ca.MX.sym('tau',Dof_act*N)  # par para articulaciones actuadas
# f_contact = ca.MX.sym('f_contact', 3*8*(N))  # fuerzas de contacto (4 por pie)
# f_i = f_contact[24*i:24*(i+1)]
# print(f_i.shape)
# J_contacts = build_Jacobian_contacts()
# print(J_contacts(q_i).T.shape)
# print( (J_contacts(q_i).T@f_i).shape)