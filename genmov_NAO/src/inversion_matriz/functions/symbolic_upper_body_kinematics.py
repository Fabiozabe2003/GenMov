from functions.symbolic_mathematical_functions import *
# --- Floating base transformation ---
def sym_floating_base(q):
    I_R_B = sTrotz(q[5]) @ sTroty(q[4]) @ sTrotx(q[3])
    I_P_B = sTrasl(q[0], q[1], q[2])
    return I_P_B @ I_R_B

# --- Forward kinematics ---
def sym_fkine_left_arm(q, qf, t):
    T_I_B = sym_floating_base(qf)
    T = sTrasl(0,   0.098,  0.1)@sTrotx(-pi/2)                               #Torso to LShoulderPitch
    T = T@sdh(0,          q[0],           0,          pi/2)                   #LShoulderPitch to LShoulderRoll
    T = T@sdh(0,          q[1]+pi/2,      0.015,      pi/2)                   #LShoulderRoll to LElbowYaw
    T = T@sdh(0.105,      q[2]+pi,        0,          pi/2)                   #LElbowYaw to LElbowRoll
    T = T@sdh(0,          q[3]+pi,        0,          pi/2)                   #LElbowRoll to LWristYaw
    T = T@sdh(0.05595 + 0.05775 ,    q[4],           0,          0) 
    return T_I_B @ T if t in ['c', 'C'] else T

def sym_fkine_left_elbow(q, qf, t):
    T_I_B = sym_floating_base(qf)
    T = sTrasl(0,   0.098,  0.1)@sTrotx(-pi/2)                               #Torso to LShoulderPitch
    T = T@sdh(0,          q[0],           0,          pi/2)                   #LShoulderPitch to LShoulderRoll
    T = T@sdh(0,          q[1]+pi/2,      0.015,      pi/2)                   #LShoulderRoll to LElbowYaw
    T = T@sdh(0.105,      q[2]+pi,        0,          pi/2)                   #LElbowYaw to LElbowRoll
    return T_I_B @ T if t in ['c', 'C'] else T

def sym_fkine_right_arm(q,qf,t):
    T_I_B=sym_floating_base(qf)
    T = sTrasl(0,   -0.098,  0.1)@sTrotx(-pi/2)                          #Torso to RShoulderPitch
    T = T@sdh(0,          q[0],           0,          pi/2)               #RShoulderPitch to RShoulderRoll
    T = T@sdh(0,          q[1]-pi/2,      0.015,      -pi/2)              #RShoulderRoll to RElbowYaw
    T = T@sdh(0.105,      q[2],        0,          pi/2)                  #RElbowYaw to RElbowRoll
    T = T@sdh(0,          q[3]+pi,        0,          pi/2)               #RElbowRoll to RWristYaw
    T = T@sdh(0.05595 + 0.05775,    q[4],           0,          0)        #RWristYaw to RHand
    return T_I_B @ T if t in ['c', 'C'] else T

def sym_fkine_right_elbow(q,qf,t):
    T_I_B=sym_floating_base(qf)
    T = sTrasl(0,   -0.098,  0.1)@sTrotx(-pi/2)                          #Torso to RShoulderPitch
    T = T@sdh(0,          q[0],           0,          pi/2)               #RShoulderPitch to RShoulderRoll
    T = T@sdh(0,          q[1]-pi/2,      0.015,      -pi/2)              #RShoulderRoll to RElbowYaw
    T = T@sdh(0.105,      q[2],        0,          pi/2)                  #RElbowYaw to RElbowRoll
    return T_I_B @ T if t in ['c', 'C'] else T
