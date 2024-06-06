import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi


def dh(d, theta, a, alpha):
  # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    T = np.array([[cos(theta),-cos(alpha)*sin(theta),sin(alpha)*sin(theta),a*cos(theta)],
                  [sin(theta),cos(alpha)*cos(theta),-sin(alpha)*cos(theta),a*sin(theta)],
                  [0,sin(alpha),cos(alpha),d],
                  [0,0,0,1]])
    return T
    
    

def fkine_ur5(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)

    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(0.089, q[0],0,pi/2) # confirmado
    T2 = dh(0, q[1],-0.425,0) # esconocido
    T3 = dh(0, q[2],-0.392,0)  # confirmado
    T4 = dh(0.109, q[3] + pi,0,-pi/2)
    T5 = dh(0.0945, q[4],0,pi/2)
    T6 = dh(0.0823, q[5],0,0)

    
    # Efector final con respecto a la base
    T = T1@T2@T3@T4@T5@T6
    return T


def jacobian_ur5(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    # Calcuar la transformacion homogenea inicial (usando q)
    T = fkine_ur5(q)

    # Iteracion para la derivada de cada articulacion (columna)
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Calcular nuevamenta la transformacion homogenea e
        # Incrementar la articulacion i-esima usando un delta
        dq[i] += delta

        # Transformacion homogenea luego del incremento (q+delta)
        T_inc = fkine_ur5(dq)

        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J


def ikine_ur5(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001

    q  = copy(q0)
    for i in range(max_iter):
        Jacobiano = jacobian_ur5(q,delta)
        f = fkine_ur5(q)[0:3,3]
        e = xdes - f
        q = q + np.dot(np.linalg.pinv(Jacobiano),e)

        if (np.linalg.norm(e)  < epsilon):
            print(f"Convergió en la iteracion {i}")
            break

    return q

def ik_gradient_ur5(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo gradiente
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
    alpha = 0.2

    q  = copy(q0)
    for i in range(max_iter):
        Jacobiano = jacobian_ur5(q,delta)
        f = fkine_ur5(q)[0:3,3] 
        e = xdes - f
        q = q + np.dot(Jacobiano.T,e)

        if (np.linalg.norm(e)  < epsilon):
            print(f"Convergió en la iteracion {i}")
            break
                
    return q
    
def jacobian_pose(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,6))
    # Implementar este Jacobiano aqui

    # Transformacion homogenea inicial (usando q)
    T = fkine_ur5(q)
    Q = rot2quat(T[0:3,0:3])
    # Iteracion para la derivada de cada columna
      # Iteracion para la derivada de cada articulacion (columna)
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Calcular nuevamenta la transformacion homogenea e
        # Incrementar la articulacion i-esima usando un delta
        dq[i] += delta

        # Transformacion homogenea luego del incremento (q+delta)
        T_inc = fkine_ur5(dq)
        Q_inc = rot2quat(T_inc[0:3,0:3])

        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
        J[3:7,i]= (Q_inc - Q)/delta
    return J

def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R

def roty(ang):
    Ry = np.array([[np.cos(ang), 0, np.sin(ang)],
                   [0, 1, 0],
                   [-np.sin(ang), 0, np.cos(ang)]])
    return Ry

def rotz(ang):
    Rz = np.array([[np.cos(ang), -np.sin(ang), 0],
                   [np.sin(ang), np.cos(ang), 0],
                   [0,0,1]])
    return Rz

def rotx(ang):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(ang), -np.sin(ang)],
                   [0, np.sin(ang), np.cos(ang)]])
    return Rx
