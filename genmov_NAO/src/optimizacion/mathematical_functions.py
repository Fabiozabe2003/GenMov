import numpy as np
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

# Función antisimétrica - antiskew
def antisimetrica(v):
    m=np.array([[0,-v[0],v[1]],
                [v[2],0,-v[0]],
                [-v[1],v[0],0]])
    return m

# Función DH
def dh(d, theta, a, alpha):
  # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    T = np.array([[cos(theta),-cos(alpha)*sin(theta),sin(alpha)*sin(theta),a*cos(theta)],
                  [sin(theta),cos(alpha)*cos(theta),-sin(alpha)*cos(theta),a*sin(theta)],
                  [0,sin(alpha),cos(alpha),d],
                  [0,0,0,1]])
    return T
