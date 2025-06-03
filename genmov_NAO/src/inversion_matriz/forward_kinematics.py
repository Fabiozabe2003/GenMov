from mathematical_functions import *
# Cinemática Directa
"""T_I_B=floating_base(qf)
def fkine_top_cam(q,t):
    T1=Trasl(0,0,0.1265) #Torso to HeadYaw
    T2=Trotz(q[0]) #HeadYaw
    T3=Troty(q[1]) #HeadPitch
    T4=Trasl(0.05871,0,0.06364) #Camara alta
    
    if t=='c' or t=='C':
        T=T_I_B@T1@T2@T3@T4
    else: 
        T =T1@T2@T3@T4
    return T

def fkine_bottom_cam(q,t):
    T1=Trasl(0,0,0.1265) #Torso to HeadYaw
    T2=Trotz(q[0]) #HeadYaw
    T3=Troty(q[1]) #HeadPitch
    T4=Trasl(0.05071,0,0.01774) #Camara baja
    if t=='c' or t=='C':
        T=T_I_B@T1@T2@T3@T4
    else: 
        T = T1@T2@T3@T4
    return T
"""
# Upper body
from upper_body_kinematics import *

# Lower body kinematics
from lower_body_kinematics import *