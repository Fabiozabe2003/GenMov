import pinocchio as pin
from pinocchio import casadi as cpin
from os.path import join
import casadi as ca


def dynamic_matrices_fun(model):
    cmodel = cpin.Model(model)
    cdata = cmodel.createData()

    # Define symbolic variables (SX type)
    cq = ca.SX.sym("q", model.nq)
    cv = ca.SX.sym("v", model.nv)

    # Compute matrices
    cpin.crba(cmodel, cdata, cq)
    M = cdata.M
    g = cpin.computeGeneralizedGravity(cmodel, cdata, cq)
    C = cpin.computeCoriolisMatrix(cmodel, cdata, cq, cv)
    b = C @ cv + g

    # Symmetrize M
    M_sym = ca.triu(M) + ca.transpose(ca.triu(M) - ca.diag(ca.diag(M)))

    # CasADi functions
    M_fun = ca.Function('M_fun', [cq], [M])
    b_fun = ca.Function('b_fun', [cq, cv], [b])
    return M_fun, b_fun


def dynamic_q(q_i):
    # Extraer componentes
    xyz = q_i[0:3]
    rpy = q_i[3:6]
    q_act = q_i[6:32]

    # Descomponer RPY
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
    cr = ca.cos(roll / 2)
    sr = ca.sin(roll / 2)
    cp = ca.cos(pitch / 2)
    sp = ca.sin(pitch / 2)
    cy = ca.cos(yaw / 2)
    sy = ca.sin(yaw / 2)

    # Calcular cuaternión 
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
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]

    #Matriz de transformación T(rpy)
    T = ca.vertcat(
        ca.horzcat(1, 0, -ca.sin(pitch)),
        ca.horzcat(0, ca.cos(roll), ca.cos(pitch)*ca.sin(roll)),
        ca.horzcat(0, -ca.sin(roll), ca.cos(pitch)*ca.cos(roll))
    )

    # T = ca.vertcat(
    #     ca.horzcat(ca.cos(pitch)*ca.cos(yaw),-ca.sin(yaw), 0),
    #     ca.horzcat(ca.cos(pitch)*ca.sin(yaw), ca.cos(yaw),0),
    #     ca.horzcat( -ca.sin(pitch),0, 1),
    # )

    omega = T @ drpy
    return omega


def dynamic_dq(dq_i, rpy):
    """
    Corrige la parte de la base de dq_i (ya ordenado base + joints),
    reemplazando drpy por omega.
    """
    dv   = dq_i[0:3]        # velocidades lineales
    drpy = dq_i[3:6]        # derivadas de RPY
    dq_joints = dq_i[6:]    # articulaciones
    
    # Compute rotation matrix from base to world (ZYX Euler)
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
    R = ca.vertcat(
        ca.horzcat(ca.cos(yaw)*ca.cos(pitch), ca.cos(yaw)*ca.sin(pitch)*ca.sin(roll)-ca.sin(yaw)*ca.cos(roll), ca.cos(yaw)*ca.sin(pitch)*ca.cos(roll)+ca.sin(yaw)*ca.sin(roll)),
        ca.horzcat(ca.sin(yaw)*ca.cos(pitch), ca.sin(yaw)*ca.sin(pitch)*ca.sin(roll)+ca.cos(yaw)*ca.cos(roll), ca.sin(yaw)*ca.sin(pitch)*ca.cos(roll)-ca.cos(yaw)*ca.sin(roll)),
        ca.horzcat(-ca.sin(pitch), ca.cos(pitch)*ca.sin(roll), ca.cos(pitch)*ca.cos(roll))
    )

    omega = rpy_to_omega(rpy, drpy)

    dv_local     = R.T @ dv
    omega_local  = omega #R.T @ omega

    dq_base_dyn = ca.vertcat(dv_local, omega_local)
    dq_dyn = ca.vertcat(dq_base_dyn, dq_joints)
    return dq_dyn


def build_dynamic_ddq():

    # Variables simbólicas
    rpy     = ca.MX.sym('rpy', 3)
    drpy    = ca.MX.sym('drpy', 3)
    ddq     = ca.MX.sym('ddq',32)

    ddx     = ddq[0:3]
    ddrpy   = ddq[3:6]
    ddq_act = ddq[6:32]


    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]

    R = ca.vertcat(
        ca.horzcat(ca.cos(yaw)*ca.cos(pitch), ca.cos(yaw)*ca.sin(pitch)*ca.sin(roll)-ca.sin(yaw)*ca.cos(roll), ca.cos(yaw)*ca.sin(pitch)*ca.cos(roll)+ca.sin(yaw)*ca.sin(roll)),
        ca.horzcat(ca.sin(yaw)*ca.cos(pitch), ca.sin(yaw)*ca.sin(pitch)*ca.sin(roll)+ca.cos(yaw)*ca.cos(roll), ca.sin(yaw)*ca.sin(pitch)*ca.cos(roll)-ca.cos(yaw)*ca.sin(roll)),
        ca.horzcat(-ca.sin(pitch), ca.cos(pitch)*ca.sin(roll), ca.cos(pitch)*ca.cos(roll))
    )

    # Matriz T(rpy)
    T = ca.vertcat(
        ca.horzcat(1, 0, -ca.sin(pitch)),
        ca.horzcat(0, ca.cos(roll), ca.cos(pitch)*ca.sin(roll)),
        ca.horzcat(0, -ca.sin(roll), ca.cos(pitch)*ca.cos(roll))
    )

    # T = ca.vertcat(
    #     ca.horzcat(ca.cos(pitch)*ca.cos(yaw),-ca.sin(yaw), 0),
    #     ca.horzcat(ca.cos(pitch)*ca.sin(yaw), ca.cos(yaw),0),
    #     ca.horzcat( -ca.sin(pitch),0, 1),
    # )


    omega = T @ drpy
    domega = ca.jacobian(omega, rpy) @ drpy + T @ ddrpy

    # Transform to local (body) frame
    ddx_local = R.T @ ddx
    domega_local =  domega #R.T @ domega

    vdot = ca.vertcat(ddx_local, domega_local, ddq_act)

    # Crear función simbólica final
    return ca.Function('dynamic_ddq', [rpy, drpy, ddq], [vdot])


# model_path = "/home/invitado8/proy_ws/src/nao/nao_utec/urdf/"
# urdf_filename = "naoV40red.urdf"
# urdf_model_path = join(model_path, urdf_filename)

# model = pin.buildModelFromUrdf(urdf_model_path, pin.JointModelFreeFlyer())
# data = model.createData()



#### PRUEBA DE FUNCIONAMIENTO
# N = 25
# Dof = 32  
# Dof_act= 26

# q = ca.MX.sym('q',Dof*N) # variable 'q' de cuero completo
# tau = ca.MX.sym('tau',Dof_act*N)  # par para articulaciones actuadas
# f_contact = ca.MX.sym('f_contact', 3*8*(N))  # fuerzas de contacto (4 por pie)

# i=0
# q_i = q[Dof*i :Dof*(i+1)]  
# q_i1 = q[Dof*(i+1):Dof*(i+2)]
# dq_i = (q_i1-q_i)/0.05
# q_i2 = q[Dof*(i+2):Dof*(i+3)]
# ddq_i = (q_i2 - 2*q_i1 + q_i)/ 0.05**2


# M_din,b_din = dynamic_matrices_fun(model)

# dyn_q = dynamic_q(q_i)  # q del modo [x y z quat qactuados]
# rpy = q_i[3:6]
# drpy = dq_i[3:6]
# ddrpy = ddq_i[3:6]
# ddx = ddq_i[0:3]
# ddq_act = ddq_i[6:]

#  # dq del modo [xdot ydot zdot wx wy wz qactuados]

# dynamic_ddq = build_dynamic_ddq()
# dyn_ddq = dynamic_ddq(rpy, drpy,ddq_i)
# print(dyn_ddq)

# M_i = M_din(dyn_q)
# print(M_i)
# b_i = b_din(dyn_q,dyn_dq)
# print(b_i)