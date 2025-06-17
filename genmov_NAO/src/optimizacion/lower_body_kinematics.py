from mathematical_functions import *

#Condiciones iniciales - base flotante c/ sistema inercial
def floating_base(q):
    I_R_B=Trotz(q[5])@Troty(q[4])@Trotx(q[3])
    I_P_B=Trasl(q[0],q[1],q[2])
    T_I_B=I_P_B@I_R_B
    return T_I_B

def fkine_left_leg(q,qf = None,t = None):
    T0=Trasl(0, 0.05, -0.085)@Trotx(-(pi/2+pi/4))   #Torso to LHipYawPitch
    T1=Trotz(q[0]) #z0
    T2=Trotx(pi/4)@Troty(pi/2)@Trotz(q[1]) #z1
    T3=Troty(-pi/2)@Trotz(q[2]) #z2
    T4=Trasl(0,0.1,0)@Trotz(q[3])
    T5=Trasl(0,0.1029,0)@Trotz(q[4])
    T6=Trotx(q[5])
    if t=='c' or t=='C':
        T_I_B=floating_base(qf)
        T=T_I_B@T0@T1@T2@T3@T4@T5@T6
    else: 
        T=T0@T1@T2@T3@T4@T5@T6
    return T

def fkine_left_knee(q,qf = None,t = None):
    T0=Trasl(0, 0.05, -0.085)@Trotx(-(pi/2+pi/4))   #Torso to LHipYawPitch
    T1=Trotz(q[0]) #z0
    T2=Trotx(pi/4)@Troty(pi/2)@Trotz(q[1]) #z1
    T3=Troty(-pi/2)@Trotz(q[2]) #z2
    T4=Trasl(0,0.1,0)@Trotz(q[3])
    if t=='c' or t=='C':
        T_I_B=floating_base(qf)
        T=T_I_B@T0@T1@T2@T3@T4
    else: 
        T=T0@T1@T2@T3@T4
    return T

def fkine_right_leg(q,qf = None,t = None):
    T0=Trasl(0, -0.05, -0.085)@Trotx(-(pi/4))   #Torso to RHipYawPitch
    T1=Trotz(q[0]) #z0
    T2=Trotx(pi/4)@Trotx(q[1]) #z1
    T3=Troty(q[2]) #z2
    T4=Trasl(0,0,-0.1)@Troty(q[3])
    T5=Trasl(0,0,-0.1029)@Troty(q[4])
    T6=Trotx(q[5])
    if t=='c' or t=='C':
        T_I_B=floating_base(qf)
        T=T_I_B@T0@T1@T2@T3@T4@T5@T6
    else: 
        T=T0@T1@T2@T3@T4@T5@T6
    return T


def fkine_right_knee(q,qf = None,t = None):
    T0=Trasl(0, -0.05, -0.085)@Trotx(-(pi/4))   #Torso to RHipYawPitch
    T1=Trotz(q[0]) #z0
    T2=Trotx(pi/4)@Trotx(q[1]) #z1
    T3=Troty(q[2]) #z2
    T4=Trasl(0,0,-0.1)@Troty(q[3])
    if t=='c' or t=='C':
        T_I_B=floating_base(qf)
        T=T_I_B@T0@T1@T2@T3@T4
    else: 
        T=T0@T1@T2@T3@T4
    return T


def fkine_left_foot_contacts(q,qf ,t):
    T0=fkine_left_leg(q,qf,t)
    T1=Trasl(-0.03,0.04519,-0.02)
    T2=Trasl(-0.03,0.04519,0.03)
    T3=Trasl(0.08,0.04519,0.03)
    T4=Trasl(0.08,0.04519,-0.02)
    return T0@T1,T0@T2,T0@T3,T0@T4

def fkine_right_foot_contacts(q,qf,t):
    T0=fkine_right_leg(q,qf,t)
    T1=Trasl(-0.03,0.04519,-0.02)
    T2=Trasl(-0.03,0.04519,0.03)
    T3=Trasl(0.08,0.04519,0.03)
    T4=Trasl(0.08,0.04519,-0.02)
    R=Trotx(-pi/2)
    return T0@R@T1,T0@R@T2,T0@R@T3,T0@R@T4


def fkine_left_foot_ground(q,qf,t):
    T0=fkine_left_leg(q,qf,t)
    T1=Trasl(-0.07,0.04519,-0.05)
    T2=Trasl(-0.07,0.04519,0.065)
    T3=Trasl(0.11,0.04519,0.065)
    T4=Trasl(0.11,0.04519,-0.05)
    return T0@T1,T0@T2,T0@T3,T0@T4

def fkine_right_foot_ground(q,qf,t):
    T0=fkine_right_leg(q,qf,t)
    T1=Trasl(-0.07,0.04519,-0.07)
    T2=Trasl(-0.07,0.04519,0.05)
    T3=Trasl(0.11,0.04519,0.05)
    T4=Trasl(0.11,0.04519,-0.07)
    R=Trotx(-pi/2)
    return T0@R@T1,T0@R@T2,T0@R@T3,T0@R@T4

def fkine_left_foot_constant(q,qf = None,t = None):
    T0=fkine_left_leg(q,qf,t)
    T1=Trasl(0.015,0.04519,0)
    return T0@T1

def fkine_right_foot_constant(q,qf = None,t = None):
    T0=fkine_right_leg(q,qf,t)
    T1=Trasl(0.015,0.04519,0)
    R=Trotx(-pi/2)
    return T0@R@T1