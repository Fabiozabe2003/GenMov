import numpy as np
from copy import copy
from forward_kinematics import *
cos=np.cos; sin=np.sin; pi=np.pi


# Functions

def q_split(q):
    q_left_arm=q[14:20]
    q_right_arm=q[20:26]
    q_left_elbow=q[14:17]
    q_right_elbow=q[20:23]
    q_left_leg=q[2:8]
    q_right_leg=q[8:14]
    q_left_knee=q[2:6]
    q_right_knee=q[8:12]

    return q_left_arm, q_right_arm, q_left_elbow, q_right_elbow, q_left_leg, q_right_leg, q_left_knee, q_right_knee

def update_positions(q,qf):
        q_left_arm, q_right_arm, q_left_elbow, q_right_elbow, q_left_leg, q_right_leg, q_left_knee, q_right_knee = q_split(q)

        # Upper body
        T_left = fkine_left_arm(q_left_arm,qf,"b"); x_left=T_left[0:3,3]
        T_right = fkine_right_arm(q_right_arm,qf,"b"); x_right=T_right[0:3,3]
        T_left_elbow = fkine_left_elbow(q_left_elbow,qf,"b"); x_left_elbow=T_left_elbow[0:3,3]
        T_right_elbow = fkine_right_elbow(q_right_elbow,qf,"b"); x_right_elbow=T_right_elbow[0:3,3]
            
        # Lower body
        T_left_leg = fkine_left_leg(q_left_leg,qf,"b"); x_left_leg=T_left_leg[0:3,3]
        T_right_leg= fkine_right_leg(q_right_leg,qf,"b"); x_right_leg=T_right_leg[0:3,3]
        T_left_knee = fkine_left_knee(q_left_knee,qf,"b"); x_left_knee=T_left_knee[0:3,3]
        T_right_knee = fkine_right_knee(q_right_knee,qf,"b"); x_right_knee=T_right_knee[0:3,3]

        return x_left,x_right,x_left_elbow,x_right_elbow,x_left_leg,x_right_leg,x_left_knee,x_right_knee

def q_ranges(q,q_min,q_max):
    for i in range(len(q)):
        #print(q_min[i],"\t",q_max[i],"\t",np.round(q[i],4),"\t",(q_min[i]<q[i] and q_max[i]>q[i]))
        if (not(q_min[i]<q[i] and q_max[i]>q[i])): print("¡Fuera de rango!")


# Jacobians
    # UPPER BODY
# Left hand
def jacobian_left_arm(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    # Calcuar la transformacion homogenea inicial (usando q)
    T = fkine_left_arm(q,'b')   
    # Iteracion para la derivada de cada articulacion (columna)
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Calcular nuevamenta la transformacion homogenea e
        # Incrementar la articulacion i-esima usando un delta
        dq[i] += delta
        # Transformacion homogenea luego del incremento (q+delta)
        T_inc = fkine_left_arm(dq,'b')
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J

def floating_base_jacobian_left_arm(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_arm(q,qf,'c')[0:3,3]-I_P_B[0:3,3])
    J3=I_R_B[0:3,0:3]@jacobian_left_arm(q)
    J=np.hstack((J1,J2,np.zeros((3,(14))),J3,np.zeros((3,6))))
    #print(J)
    return J

# Left elbow
def jacobian_left_elbow(q, delta=0.0001):
    J = np.zeros((3,3))
    T = fkine_left_elbow(q,'b')   
    for i in range(3):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_left_elbow(dq,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J

def floating_base_jacobian_left_elbow(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_elbow(q,qf,'c')[0:3,3]-I_P_B[0:3,3])
    J3=I_R_B[0:3,0:3]@jacobian_left_elbow(q)
    J=np.hstack((J1,J2,np.zeros((3,14)),J3,np.zeros((3,9))))
    return J

# Right hand
def jacobian_right_arm(q, delta=1e-6):
    J = np.zeros((3,6))
    T = fkine_right_arm(q,'b')
    for i in range(6):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_right_arm(dq,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_right_arm(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_arm(q,qf,'c')[0:3,3]-I_P_B[0:3,3])
    J3=I_R_B[0:3,0:3]@jacobian_right_arm(q)
    J=np.hstack((J1,J2,np.zeros((3,20)),J3))
    return J

# Right elbow
def jacobian_right_elbow(q, delta=0.0001):
    J = np.zeros((3,3))
    T = fkine_right_elbow(q,'b')   
    for i in range(3):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_right_elbow(dq,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J

def floating_base_jacobian_right_elbow(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_elbow(q,qf,'c')[0:3,3]-I_P_B[0:3,3])
    J3=I_R_B[0:3,0:3]@jacobian_right_elbow(q)
    J=np.hstack((J1,J2,np.zeros((3,20)),J3,np.zeros((3,3))))
    return J





# LOWER BODY
def jacobian_left_leg(q, delta=0.0001):
    J = np.zeros((3,6))
    T = fkine_left_leg(q,'b')   
    for i in range(6):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_left_leg(dq,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_left_leg(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_leg(q,qf,'c')[0:3,3]-I_P_B[0:3,3])
    J3=I_R_B[0:3,0:3]@jacobian_left_leg(q)
    J=np.hstack((J1,J2,np.zeros((3,2)),J3,np.zeros((3,18))))
    return J



def jacobian_right_leg(q, delta=0.0001):
    J = np.zeros((3,6))
    T = fkine_right_leg(q,'b')   
    for i in range(6):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_right_leg(dq,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_right_leg(q,qf):
    #J=np.array([np.eye(3), -antisimetrica(fkine_right_leg(q,'c')[0:3,3]-I_P_B[0:3]), I_R_B@jacobian_right_leg(q)])
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_leg(q,qf,'c')[0:3,3]-I_P_B[0:3,3])
    J3=I_R_B[0:3,0:3]@jacobian_right_leg(q)
    J=np.hstack((J1,J2,np.zeros((3,8)),J3,np.zeros((3,12))))
    return J


def jacobian_left_knee(q, delta=0.0001):
    J = np.zeros((3,4))
    T = fkine_left_knee(q,'b')   
    for i in range(4):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_left_knee(dq,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_left_knee(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_knee(q,qf,'c')[0:3,3]-I_P_B[0:3,3])
    J3=I_R_B[0:3,0:3]@jacobian_left_knee(q)
    J=np.hstack((J1,J2,np.zeros((3,2)),J3,np.zeros((3,20))))
    return J

def jacobian_right_knee(q, delta=0.0001):
    J = np.zeros((3,4))
    T = fkine_right_knee(q,'b')   
    for i in range(4):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_right_knee(dq,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_right_knee(q,qf):
    #J=np.array([np.eye(3), -antisimetrica(fkine_right_leg(q,'c')[0:3,3]-I_P_B[0:3]), I_R_B@jacobian_right_leg(q)])
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_knee(q,qf,'c')[0:3,3]-I_P_B[0:3,3])
    J3=I_R_B[0:3,0:3]@jacobian_right_knee(q)
    J=np.hstack((J1,J2,np.zeros((3,8)),J3,np.zeros((3,14))))
    return J


def jacobian_left_foot_contacts(q, delta=0.0001):
    J = [np.zeros((3,6)) for _ in range(4)]
    T = fkine_left_foot_contacts(q,'b')  
    for j in range(4):
        for i in range(6):
            dq = copy(q)
            dq[i] += delta
            Ts = fkine_left_foot_contacts(dq,'b')
            T_inc=Ts[j]
            J[j][0:3,i]=(T_inc[0:3,3]-T[j][0:3,3])/delta
    return J

def floating_base_jacobian_left_foot_contacts(q,qf):
    J = [np.zeros((3,6)) for _ in range(4)]
    for i in range (4):
        J1=np.eye(3)
        J2=-antisimetrica(fkine_left_foot_contacts(q,qf,'c')[i][0:3,3]-I_P_B[0:3,3])
        J3=I_R_B[0:3,0:3]@(jacobian_left_foot_contacts(q))[i]
        J[i]=np.hstack((J1,J2,np.zeros((3,2)),J3,np.zeros((3,18))))
    J=np.vstack((J[0],J[1],J[2],J[3]))
    print(J)
    return J

def jacobian_right_foot_contacts(q, delta=0.0001):
    J = [np.zeros((3,6)) for _ in range(4)]
    T = fkine_right_foot_contacts(q,'b')  
    for j in range(4):
        for i in range(6):
            dq = copy(q)
            dq[i] += delta
            Ts = fkine_right_foot_contacts(dq,'b')
            T_inc=Ts[j]
            J[j][0:3,i]=(T_inc[0:3,3]-T[j][0:3,3])/delta
    return J

def floating_base_jacobian_right_foot_contacts(q,qf):
    J = [np.zeros((3,6)) for _ in range(4)]
    for i in range (4):
        J1=np.eye(3)
        J2=-antisimetrica(fkine_right_foot_contacts(q,qf,'c')[i][0:3,3]-I_P_B[0:3,3])
        J3=I_R_B[0:3,0:3]@(jacobian_right_foot_contacts(q))[i]
        J[i]=np.hstack((J1,J2,np.zeros((3,2)),J3,np.zeros((3,20))))
    J=np.vstack((J[0],J[1],J[2],J[3]))
    return J