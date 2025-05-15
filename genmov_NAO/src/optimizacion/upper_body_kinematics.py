from mathematical_functions import *

#Condiciones iniciales - base flotante c/ sistema inercial
def floating_base(q):
    I_R_B=Trotz(q[5])@Troty(q[4])@Trotx(q[3])
    I_P_B=Trasl(q[0],q[1],q[2])
    T_I_B=I_P_B@I_R_B
    return T_I_B

def fkine_left_arm(q,qf = None,t = None):
    T0 = Trasl(0,   0.098,  0.1)@Trotx(-pi/2)                               #Torso to LShoulderPitch
    T1 = dh(0,          q[0],           0,          pi/2)                   #LShoulderPitch to LShoulderRoll
    T2 = dh(0,          q[1]+pi/2,      0.015,      pi/2)                   #LShoulderRoll to LElbowYaw
    T3 = dh(0.105,      q[2]+pi,        0,          pi/2)                   #LElbowYaw to LElbowRoll
    T4 = dh(0,          q[3]+pi,        0,          pi/2)                   #LElbowRoll to LWristYaw
    T5 = dh(0.05595 + 0.05775 ,    q[4],           0,          0)           #LWristYaw to LHand
    if t=='c' or t=='C':
        T_I_B=floating_base(qf)
        T=T_I_B@T0@T1@T2@T3@T4@T5
    else: 
        T=T0@T1@T2@T3@T4@T5
    return T

def fkine_left_elbow(q,qf = None, t = None):
    T0 = Trasl(0,   0.098,  0.1)@Trotx(-pi/2)                               #Torso to LShoulderPitch
    T1 = dh(0,          q[0],           0,          pi/2)                   #LShoulderPitch to LShoulderRoll
    T2 = dh(0,          q[1]+pi/2,      0.015,      pi/2)                   #LShoulderRoll to LElbowYaw
    T3 = dh(0.105,      q[2]+pi,        0,          pi/2)                   #LElbowYaw to LElbowRoll
    if t=='c' or t=='C':
        T_I_B=floating_base(qf)
        T=T_I_B@T0@T1@T2@T3
    else: 
        T=T0@T1@T2@T3
    return T

# DH brazo derecho
def fkine_right_arm(q,qf = None,t = None):
    T0 = Trasl(0,   -0.098,  0.1)@Trotx(-pi/2)                          #Torso to RShoulderPitch
    T1 = dh(0,          q[0],           0,          pi/2)               #RShoulderPitch to RShoulderRoll
    T2 = dh(0,          q[1]-pi/2,      0.015,      -pi/2)              #RShoulderRoll to RElbowYaw
    T3 = dh(0.105,      q[2],        0,          pi/2)                  #RElbowYaw to RElbowRoll
    T4 = dh(0,          q[3]+pi,        0,          pi/2)               #RElbowRoll to RWristYaw
    T5 = dh(0.05595 + 0.05775,    q[4],           0,          0)        #RWristYaw to RHand
    if t=='c' or t=='C':
        T_I_B=floating_base(qf)
        T=T_I_B@T0@T1@T2@T3@T4@T5
    else: 
        T=T0@T1@T2@T3@T4@T5
    return T

def fkine_right_elbow(q,qf = None,t = None):

    T0 = Trasl(0,   -0.098,  0.1)@Trotx(-pi/2)                          #Torso to RShoulderPitch
    T1 = dh(0,          q[0],           0,          pi/2)               #RShoulderPitch to RShoulderRoll
    T2 = dh(0,          q[1]-pi/2,      0.015,      -pi/2)              #RShoulderRoll to RElbowYaw
    T3 = dh(0.105,      q[2],        0,          pi/2)                  #RElbowYaw to RElbowRoll
    if t=='c' or t=='C':
        T_I_B=floating_base(qf)
        T=T_I_B@T0@T1@T2@T3
    else: 
        T=T0@T1@T2@T3
    return T