import casadi as ca
cos=ca.cos; sin=ca.sin; pi=ca.pi

# Funciones de rotación
def sTrotx(ang):
    c = cos(ang)
    s = sin(ang)
    Tx = ca.vertcat(
        ca.horzcat(1, 0, 0, 0),
        ca.horzcat(0, c, -s, 0),
        ca.horzcat(0, s,  c, 0),
        ca.horzcat(0, 0, 0, 1)
    )
    return Tx
def sTroty(ang):
    c = cos(ang)
    s = sin(ang)
    Tx = ca.vertcat(
        ca.horzcat(c, 0, s, 0),
        ca.horzcat(0, 1, 0, 0),
        ca.horzcat(-s, 0, c, 0),
        ca.horzcat(0, 0, 0, 1)
    )
    return Tx
def sTrotz(ang):
    c = cos(ang)
    s = sin(ang)
    Tx = ca.vertcat(
        ca.horzcat(c, -s, 0, 0),
        ca.horzcat(s, c, 0, 0),
        ca.horzcat(0, 0,  1, 0),
        ca.horzcat(0, 0, 0, 1)
    )
    return Tx

# Función Traslación
def sTrasl(x,y,z):
    T=ca.vertcat(
        ca.horzcat(1,0,0,x),
        ca.horzcat(0,1,0,y),
        ca.horzcat(0,0,1,z),
        ca.horzcat(0,0,0,1)
    )
    return T
def sAntisimetrica(v):
    return ca.vertcat(
        ca.horzcat(0,     -v[2],  v[1]),
        ca.horzcat(v[2],   0,    -v[0]),
        ca.horzcat(-v[1],  v[0],  0)
    )
def sdh(d, theta, a, alpha):
    cth = ca.cos(theta)
    sth = ca.sin(theta)
    ca_ = ca.cos(alpha)
    sa_ = ca.sin(alpha)
    T = ca.vertcat(
        ca.horzcat(cth, -ca_ * sth, sa_ * sth, a * cth),
        ca.horzcat(sth,  ca_ * cth, -sa_ * cth, a * sth),
        ca.horzcat(0,    sa_,       ca_,       d),
        ca.horzcat(0,    0,         0,         1)
    )
    return T


def dynamic_q(q_i):
    # Extraer componentes
    xyz = q_i[26:29]
    rpy = q_i[29:32]
    q_act = q_i[0:26]
    # Descomponer RPY
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
    cr = ca.cos(roll / 2)
    sr = ca.sin(roll / 2)
    cp = ca.cos(pitch / 2)
    sp = ca.sin(pitch / 2)
    cy = ca.cos(yaw / 2)
    sy = ca.sin(yaw / 2)
    # Calcular cuaternión (qw, qx, qy, qz)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    quat = ca.vertcat(qx, qy, qz, qw)
    # Reordenar: [xyz, quat, q_act]
    q_dyn = ca.vertcat(xyz, quat, q_act)
    return q_dyn

def rpy_to_omega(rpy, drpy):
    """
    Transforma la derivada de RPY (drpy) en velocidad angular omega.
    """
    roll=rpy[0]; pitch=rpy[1]; yaw = rpy[2]

    # Matriz de transformación T(rpy)
    T = ca.vertcat(
        ca.horzcat(1, 0, -ca.sin(pitch)),
        ca.horzcat(0, ca.cos(roll), ca.cos(pitch)*ca.sin(roll)),
        ca.horzcat(0, -ca.sin(roll), ca.cos(pitch)*ca.cos(roll))
    )

    omega = T @ drpy
    return omega

def dynamic_dq(dq_i, rpy):
    """
    Corrige la parte de la base de dq_i (ya ordenado base + joints),
    reemplazando drpy por omega.
    """
    dv   = dq_i[26:29]        # velocidades lineales
    drpy = dq_i[29:32]        # derivadas de RPY
    dq_joints = dq_i[0:26]    # articulaciones

    omega = rpy_to_omega(rpy, drpy)
    dq_base_dyn = ca.vertcat(dv, omega)
    
    dq_dyn = ca.vertcat(dq_base_dyn, dq_joints)
    return dq_dyn


def sR2RPY(R):
    """
    Convert a 3x3 rotation matrix R to roll, pitch, and yaw (RPY) angles.

    Args:
    R: 3x3 CasADi MX matrix (rotation matrix)

    Returns:
    rpy: 3x1 CasADi MX matrix (roll, pitch, yaw angles)
    """
    roll = ca.atan2(R[2,1], R[2,2])  # Roll (phi)
    pitch = ca.atan2(-R[2,0], ca.mtimes(ca.mtimes(R[0,0], R[0,0]), R[1,0]))  # Pitch (theta)
    yaw = ca.atan2(R[1,0], R[0,0])   # Yaw (psi)

    rpy = ca.vertcat(roll, pitch, yaw)
    return rpy