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
# Left hand
def jacobian_left_arm(q, qf, delta=0.0001):
    """Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]"""
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    # Calcuar la transformacion homogenea inicial (usando q)
    T = fkine_left_arm(q, qf,'b')   
    # Iteracion para la derivada de cada articulacion (columna)
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Calcular nuevamenta la transformacion homogenea e
        # Incrementar la articulacion i-esima usando un delta
        dq[i] += delta
        # Transformacion homogenea luego del incremento (q+delta)
        T_inc = fkine_left_arm(dq, qf,'b')
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_left_arm(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_arm(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@jacobian_left_arm(q, qf)
    J=np.hstack((np.zeros((3,(14))),J3,np.zeros((3,6)),J1,J2))
    return J

# Left elbow
def jacobian_left_elbow(q, qf, delta=0.0001):
    J = np.zeros((3,3))
    T = fkine_left_elbow(q, qf,'b')   
    for i in range(3):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_left_elbow(dq, qf,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_left_elbow(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_elbow(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@jacobian_left_elbow(q, qf)
    J=np.hstack((np.zeros((3,14)),J3,np.zeros((3,9)),J1,J2))
    return J

# Right hand
def jacobian_right_arm(q, qf, delta=1e-6):
    J = np.zeros((3,6))
    T = fkine_right_arm(q, qf,'b')
    for i in range(6):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_right_arm(dq, qf,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_right_arm(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_arm(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@jacobian_right_arm(q, qf)
    J=np.hstack((np.zeros((3,20)),J3,J1,J2))
    return J

# Right elbow
def jacobian_right_elbow(q, qf, delta=0.0001):
    J = np.zeros((3,3))
    T = fkine_right_elbow(q, qf,'b')   
    for i in range(3):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_right_elbow(dq, qf,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_right_elbow(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_elbow(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@jacobian_right_elbow(q, qf)
    J=np.hstack((np.zeros((3,20)),J3,np.zeros((3,3)),J1,J2))
    return J

# LOWER BODY
def jacobian_left_leg(q, qf, delta=0.0001):
    J = np.zeros((3,6))
    T = fkine_left_leg(q, qf,'b')   
    for i in range(6):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_left_leg(dq, qf,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_left_leg(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_leg(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@jacobian_left_leg(q, qf)
    J=np.hstack((np.zeros((3,2)),J3,np.zeros((3,18)),J1,J2))
    return J

def jacobian_right_leg(q, qf, delta=0.0001):
    J = np.zeros((3,6))
    T = fkine_right_leg(q, qf,'b')   
    for i in range(6):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_right_leg(dq, qf,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_right_leg(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_leg(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@jacobian_right_leg(q, qf)
    J=np.hstack((np.zeros((3,8)),J3,np.zeros((3,12)),J1,J2))
    return J

def jacobian_left_knee(q, qf, delta=0.0001):
    J = np.zeros((3,4))
    T = fkine_left_knee(q, qf,'b')   
    for i in range(4):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_left_knee(dq, qf,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_left_knee(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_knee(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@jacobian_left_knee(q, qf)
    J=np.hstack((np.zeros((3,2)),J3,np.zeros((3,20)),J1,J2))
    return J

def jacobian_right_knee(q, qf, delta=0.0001):
    J = np.zeros((3,4))
    T = fkine_right_knee(q, qf,'b')   
    for i in range(4):
        dq = copy(q)
        dq[i] += delta
        T_inc = fkine_right_knee(dq, qf,'b')
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J
def floating_base_jacobian_right_knee(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_knee(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@jacobian_right_knee(q, qf)
    J=np.hstack((np.zeros((3,8)),J3,np.zeros((3,14)),J1,J2))
    return J

def jacobian_left_foot_contacts(q, qf, delta=0.0001):
    J = [np.zeros((3,6)) for _ in range(4)]
    T = fkine_left_foot_contacts(q, qf,'b')  
    for j in range(4):
        for i in range(6):
            dq = copy(q)
            dq[i] += delta
            Ts = fkine_left_foot_contacts(dq, qf,'b')
            T_inc=Ts[j]
            J[j][0:3,i]=(T_inc[0:3,3]-T[j][0:3,3])/delta
    return J
def floating_base_jacobian_left_foot_contacts(q,qf):
    J = [np.zeros((3,6)) for _ in range(4)]
    for i in range (4):
        J1=np.eye(3)
        J2=-antisimetrica(fkine_left_foot_contacts(q,qf,'c')[i][0:3,3]-floating_base(qf)[0:3,3])
        J3=floating_base(qf)[0:3,0:3]@(jacobian_left_foot_contacts(q, qf))[i]
        J[i]=np.hstack((np.zeros((3,2)),J3,np.zeros((3,18)),J1,J2))
    J=np.vstack((J[0],J[1],J[2],J[3]))
    return J

def jacobian_right_foot_contacts(q, qf, delta=0.0001):
    J = [np.zeros((3,6)) for _ in range(4)]
    T = fkine_right_foot_contacts(q, qf,'b')  
    for j in range(4):
        for i in range(6):
            dq = copy(q)
            dq[i] += delta
            Ts = fkine_right_foot_contacts(dq, qf,'b')
            T_inc=Ts[j]
            J[j][0:3,i]=(T_inc[0:3,3]-T[j][0:3,3])/delta
    return J
def floating_base_jacobian_right_foot_contacts(q,qf):
    J = [np.zeros((3,6)) for _ in range(4)]
    for i in range (4):
        J1=np.eye(3)
        J2=-antisimetrica(fkine_right_foot_contacts(q,qf,'c')[i][0:3,3]-floating_base(qf)[0:3,3])
        J3=floating_base(qf)[0:3,0:3]@(jacobian_right_foot_contacts(q, qf))[i]
        J[i]=np.hstack((np.zeros((3,8)),J3,np.zeros((3,12)),J1,J2))
    J=np.vstack((J[0],J[1],J[2],J[3]))
    return J

def jacobian_left_foot_constant(q, qf, delta=0.0001):
    J = np.zeros((6,6))
    T=fkine_left_foot_constant(q, qf,'b')
    for i in range(6):
        dq=copy(q)
        dq[i] += delta
        T_inc=fkine_left_foot_constant(dq, qf,'b')
        dp = (T_inc[0:3,3]-T[0:3,3])/delta
        dR=(T[0:3,0:3]).T @ T_inc[0:3,0:3]
        dtheta = np.array([
                dR[2,1] - dR[1,2],
                dR[0,2] - dR[2,0],
                dR[1,0] - dR[0,1]
            ]) / 2.0
        domega = dtheta / delta
        J[0:3,i]=dp
        J[3:6,i]=domega
        return J 
def floating_base_jacobian_left_foot_constant(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_left_foot_constant(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@(jacobian_left_foot_constant(q, qf))[0:3,:]
    Js=np.hstack((np.zeros((3,2)),J3,np.zeros((3,18)),J1,J2))
    J1=np.zeros((3))
    roll=qf[3], pitch=qf[4], yaw=qf[5]
    J2=np.array([[1, 0, -sin(pitch)],
                 [0, cos(roll), sin(roll)*cos(pitch)],
                 [0, -sin(roll), cos(roll)*cos(pitch)]
                 ])
    J3=floating_base(qf)[0:3,0:3]@(jacobian_left_foot_contacts(q, qf))[3:6,:]
    Ji=np.hstack((np.zeros((3,2)),J3,np.zeros((3,18)),J1,J2))
    J=np.vstack((Js,Ji))
    return J

def jacobian_right_foot_constant(q, qf, delta=0.0001):
    J = np.zeros((6,6))
    T=fkine_right_foot_constant(q, qf,'b')
    for i in range(6):
        dq=copy(q)
        dq[i] += delta
        T_inc=fkine_right_foot_constant(dq, qf,'b')
        dp = (T_inc[0:3,3]-T[0:3,3])/delta
        dR=(T[0:3,0:3]).T @ T_inc[0:3,0:3]
        dtheta = np.array([
                dR[2,1] - dR[1,2],
                dR[0,2] - dR[2,0],
                dR[1,0] - dR[0,1]
            ]) / 2.0
        domega = dtheta / delta
        J[0:3,i]=dp
        J[3:6,i]=domega
        return J
def floating_base_jacobian_right_foot_constant(q,qf):
    J1=np.eye(3)
    J2=-antisimetrica(fkine_right_foot_constant(q,qf,'c')[0:3,3]-floating_base(qf)[0:3,3])
    J3=floating_base(qf)[0:3,0:3]@(jacobian_right_foot_constant(q, qf))[0:3,:]
    Js=np.hstack((np.zeros((3,8)),J3,np.zeros((3,12)),J1,J2))
    J1=np.zeros((3))
    roll=qf[3], pitch=qf[4], yaw=qf[5]
    J2=np.array([[1, 0, -sin(pitch)],
                 [0, cos(roll), sin(roll)*cos(pitch)],
                 [0, -sin(roll), cos(roll)*cos(pitch)]
                 ])
    J3=floating_base(qf)[0:3,0:3]@(jacobian_right_foot_constant(q, qf))[3:6,:]
    Ji=np.hstack((np.zeros((3,8)),J3,np.zeros((3,12)),J1,J2))
    J=np.vstack((Js,Ji))
    return J