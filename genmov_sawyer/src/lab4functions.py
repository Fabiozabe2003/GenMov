import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi

#CON EL SAWYER AHORA ES 7 EN LUGAR DE 6
def dh(d, theta, a, alpha):
  # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    T = np.array([[cos(theta),-cos(alpha)*sin(theta),sin(alpha)*sin(theta),a*cos(theta)],
                  [sin(theta),cos(alpha)*cos(theta),-sin(alpha)*cos(theta),a*sin(theta)],
                  [0,sin(alpha),cos(alpha),d],
                  [0,0,0,1]])
    return T
    
    

def fkine_sawyer(q):
    """
    Calcular la cinematica directa del robot Sawyer dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6, q7]
    """
    # Longitudes (en metros)
    T1 = dh(0.317,      q[0],          0.081,  -pi/2)
    T2 = dh(0.194,      q[1]+3*pi/2,   0,      -pi/2)
    T3 = dh(0.4,        q[2],          0,      -pi/2)
    T4 = dh(0.1685,     q[3]+pi,       0,      -pi/2)
    T5 = dh(0.4,        q[4],          0,      -pi/2)
    T6 = dh(0.1363,     q[5]+pi,       0,      -pi/2)
    T7 = dh(0.13475,    q[6]+3*pi/2,   0,      0)
    
    #Shiwei Wang
    """T1 = dh(0.237,      q[0],   0.081,  -pi/2)
    T2 = dh(0.1925,     q[1],   0,      -pi/2)
    T3 = dh(0.4,        q[2],   0,      -pi/2)
    T4 = dh(-0.1685,    q[3],   0,      -pi/2)
    T5 = dh(0.4,        q[4],   0,      -pi/2)
    T6 = dh(0.1363,     q[5],   0,      -pi/2)
    T7 = dh(0.11,       q[6],   0,      0)"""

    #Elaboración propia
    """T1 = dh(0.0,      q[0]+pi,   -0.081,  -pi/2)
    T2 = dh(0.1925,     q[1]-pi/2,   0,      pi/2)
    T3 = dh(0.400,        q[2],   0,      pi/2)
    T4 = dh(0.1685,    q[3]+pi,   0,      -pi/2)
    T5 = dh(0.400,        q[4],   0,      pi/2)
    T6 = dh(0.1363,     q[5]+pi,   0,      pi/2)
    T7 = dh(0.13375,       q[6],   0,      0)"""

    # Efector final con respecto a la base
    T = T1@T2@T3@T4@T5@T6@T7
    return T


def jacobian_sawyer(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x7 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6, q7]
    """
    # Crear una matriz 3x7
    J = np.zeros((3,7))
    # Calcuar la transformacion homogenea inicial (usando q)
    T = fkine_sawyer(q)

    
    # Iteracion para la derivada de cada articulacion (columna)
    for i in range(7):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Calcular nuevamenta la transformacion homogenea e
        # Incrementar la articulacion i-esima usando un delta
        dq[i] += delta

        # Transformacion homogenea luego del incremento (q+delta)
        T_inc = fkine_sawyer(dq)


        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
    
    return J


def ikine_sawyer(xdes, q0):
    """
    Calcular la cinematica inversa de sawyer numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001

    q  = copy(q0)
    for i in range(max_iter):
        Jacobiano = jacobian_sawyer(q,delta)
        f = fkine_sawyer(q)[0:3,3]
        e = xdes - f
        q = q + np.dot(np.linalg.pinv(Jacobiano),e)

        if (np.linalg.norm(e)  < epsilon):
            print(f"Convergió en la iteracion {i}")
            break

    return q

def ik_gradient_sawyer(xdes, q0):
    """
    Calcular la cinematica inversa de sawyer numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo gradiente
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
    alpha = 0.2

    q  = copy(q0)
    for i in range(max_iter):
        Jacobiano = jacobian_sawyer(q,delta)
        f = fkine_sawyer(q)[0:3,3] 
        e = xdes - f
        q = q + np.dot(Jacobiano.T,e)

        if (np.linalg.norm(e)  < epsilon):
            print(f"Convergió en la iteracion {i}")
            break
                
    return q
