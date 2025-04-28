import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi

# Funciones de Rotación
def Trotx(ang):
    Tx = np.array([[1, 0, 0, 0],
                   [0, cos(ang), -sin(ang), 0],
                   [0, sin(ang), cos(ang), 0],
                   [0, 0, 0, 1]])
    return Tx

def Troty(ang):
    Ty = np.array([[cos(ang), 0, sin(ang), 0],
                   [0, 1, 0, 0],
                   [-sin(ang), 0, cos(ang), 0],
                   [0, 0, 0, 1]])
    return Ty

def Trotz(ang):
    Tz = np.array([[cos(ang), -sin(ang), 0, 0],
                   [sin(ang), cos(ang), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    return Tz

# Función Traslación
def Trasl(x,y,z):
    T = np.array([[1, 0, 0, x],
                  [0, 1, 0, y],
                  [0, 0, 1, z],
                  [0, 0, 0, 1]])
    return T

# Función DH
def dh(d, theta, a, alpha):
  # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    T = np.array([[cos(theta),-cos(alpha)*sin(theta),sin(alpha)*sin(theta),a*cos(theta)],
                  [sin(theta),cos(alpha)*cos(theta),-sin(alpha)*cos(theta),a*sin(theta)],
                  [0,sin(alpha),cos(alpha),d],
                  [0,0,0,1]])
    return T
    
# DHs
# DH cabeza (está mal)
def fkine_head(q):
    T1=Trasl(0,0,0.1265)
    T2=dh(0.0, q[0], 0.0, 0.0)
    T3=dh(0.0, q[1]-pi/2, 0, -pi/2)
    T4=Trotx(pi/2)@Troty(pi/2)
    T5=Trasl(0.0539,0,0.0679)
    T = T1@T2@T3@T4@T5
    return T

# DH brazo izquierdo
def fkine_left_arm(q):
    #DH: A simple and fast ...
    
    T0 = Trasl(0,   0.098,  0.1)@Trotx(-pi/2)                          #Torso to LShoulderPitch
    T1 = dh(0,          q[0],           0,          pi/2)       #LShoulderPitch to LShoulderRoll
    T2 = dh(0,          q[1]+pi/2,      0.015,      pi/2)       #LShoulderRoll to LElbowYaw
    T3 = dh(0.105,      q[2]+pi,        0,          pi/2)       #LElbowYaw to LElbowRoll
    T4 = dh(0,          q[3]+pi,        0,          pi/2)       #LElbowRoll to LWristYaw
    T5 = dh(0.05595 + 0.05775 ,    q[4],           0,          0)          #LWristYaw to LHand
    

    # Efector final con respecto a la base
    T = T0@T1@T2@T3@T4@T5
    return T

# DH brazo derecho
def fkine_right_arm(q):
    #DH: A simple and fast ...
        
    T0 = Trasl(0,   -0.098,  0.1)@Trotx(-pi/2)                          #Torso to RShoulderPitch
    T1 = dh(0,          q[0],           0,          pi/2)       #RShoulderPitch to RShoulderRoll
    T2 = dh(0,          q[1]-pi/2,      0.015,      -pi/2)       #RShoulderRoll to RElbowYaw
    T3 = dh(0.105,      q[2],        0,          pi/2)       #RElbowYaw to RElbowRoll
    T4 = dh(0,          q[3]+pi,        0,          pi/2)       #RElbowRoll to RWristYaw
    T5 = dh(0.05595 + 0.05775,    q[4],           0,          0)          #RWristYaw to RHand
    

    # Efector final con respecto a la base
    T = T0@T1@T2@T3@T4@T5
    return T

def fkine_right_arm(q):
    #DH: A simple and fast ...
        
    T0 = Trasl(0,   -0.098,  0.1)@Trotx(-pi/2)                          #Torso to RShoulderPitch
    T1 = dh(0,          q[0],           0,          pi/2)       #RShoulderPitch to RShoulderRoll
    T2 = dh(0,          q[1]-pi/2,      0.015,      -pi/2)       #RShoulderRoll to RElbowYaw
    T3 = dh(0.105,      q[2],        0,          pi/2)       #RElbowYaw to RElbowRoll
    T4 = dh(0,          q[3]+pi,        0,          pi/2)       #RElbowRoll to RWristYaw
    T5 = dh(0.05595 + 0.05775,    q[4],           0,          0)          #RWristYaw to RHand
    

    # Efector final con respecto a la base
    T = T0@T1@T2@T3@T4@T5
    return T

def fkine_right_elbow(q):
    #DH: A simple and fast ...
        
    T0 = Trasl(0,   -0.098,  0.1)@Trotx(-pi/2)                          #Torso to RShoulderPitch
    T1 = dh(0,          q[0],           0,          pi/2)       #RShoulderPitch to RShoulderRoll
    T2 = dh(0,          q[1]-pi/2,      0.015,      -pi/2)       #RShoulderRoll to RElbowYaw
    T3 = dh(0.105,      q[2],        0,          pi/2)       #RElbowYaw to RElbowRoll
    T4 = dh(0,          q[3]+pi,        0,          pi/2)       #RElbowRoll to RWristYaw
    T5 = dh(0.05595 + 0.05775,    q[4],           0,          0)          #RWristYaw to RHand
    

    # Efector final con respecto a la base
    T = T0@T1@T2@T3@T4@T5
    return T

def jacobian_left_arm(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    # Calcuar la transformacion homogenea inicial (usando q)
    T = fkine_left_arm(q)
    
    # Iteracion para la derivada de cada articulacion (columna)
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)

        # Calcular nuevamenta la transformacion homogenea e
        # Incrementar la articulacion i-esima usando un delta
        dq[i] += delta

        # Transformacion homogenea luego del incremento (q+delta)
        T_inc = fkine_left_arm(dq)

        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J


def jacobian_right_arm(q, delta=1e-6):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x5
    J = np.zeros((3,5))
    # Calcuar la transformacion homogenea inicial (usando q)
    T = fkine_right_arm(q)
    
    # Iteracion para la derivada de cada articulacion (columna)
    for i in range(5):
        # Copiar la configuracion articular inicial
        dq = copy(q)

        # Calcular nuevamenta la transformacion homogenea e
        # Incrementar la articulacion i-esima usando un delta
        dq[i] += delta

        # Transformacion homogenea luego del incremento (q+delta)
        T_inc = fkine_right_arm(dq)

        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J

def jacobian_right_elbow(q, delta=1e-6):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x5
    J = np.zeros((3,5))
    # Calcuar la transformacion homogenea inicial (usando q)
    T = fkine_right_elbow(q)
    
    # Iteracion para la derivada de cada articulacion (columna)
    for i in range(3):
        # Copiar la configuracion articular inicial
        dq = copy(q)

        # Calcular nuevamenta la transformacion homogenea e
        # Incrementar la articulacion i-esima usando un delta
        dq[i] += delta

        # Transformacion homogenea luego del incremento (q+delta)
        T_inc = fkine_right_elbow(dq)

        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    return J


def ikine_left_arm(xdes, q0):
    """
    Calcular la cinematica inversa de sawyer numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001

    q  = copy(q0)
    for i in range(max_iter):
        Jacobiano = jacobian_left_arm(q,delta)
        f = jacobian_left_arm(q)[0:3,3]
        e = xdes - f
        q = q + np.dot(np.linalg.pinv(Jacobiano),e)

        if (np.linalg.norm(e)  < epsilon):
            print(f"Convergió en la iteracion {i}")
            break

    return q