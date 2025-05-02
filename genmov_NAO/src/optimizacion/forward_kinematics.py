from mathematical_functions import *
# Upper body
from upper_body_kinematics import *

# Lower body kinematics
from lower_body_kinematics import *


#Condiciones iniciales - base flotante c/ sistema inercial
I_R_B= np.array([[1, 0, 0,0],
                   [0, 1,0,0],
                   [0, 0,1,0],
                   [0,0,0,1]])

x_0=0;  y_0=0;  z_0=0
I_P_B=Trasl(x_0,y_0,z_0)
T_I_B=I_P_B@I_R_B

# Cinemática Directa
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

