import numpy as np
import casadi as ca

def joint_contraints(dt,dof,q,dq,ddq,q_min,q_max,dq_min,dq_max,constraints):
    # Position constraint
    for i in range(dof):
        constraint_1 = q_min[i] <= q[i] + dq[i] * dt + 0.5 * ddq[i] * dt**2
        constraint_2 = q[i] + dq[i] * dt + 0.5 * ddq[i] * dt**2 <= q_max[i]

        constraints.append(constraint_1)
        constraints.append(constraint_2)
    
    # Velocity contraint
    """for i in range(dof):
        constraints+=[
            dq_min[i] <= dq[i] + ddq[i] * dt,
            dq[i] + ddq[i] * dt  <= dq_max[i]
            ]"""
    
    return constraints

# Limits
# http://doc.aldebaran.com/2-1/family/robots/joints_robot.html
q_min = [-2.086017, -0.330041, #Chequear Head-Yaw/Pitch
    -1.145303, -0.379472, -1.535889, -0.092346, -1.189516, -0.397880, #Chequear ankle-pitch/roll
    -1.145303, -0.790447, -1.535889, -0.103083, -1.186448, -0.768992, #Chequear ankle-pitch/roll
    -2.0857 , -0.3142, -2.0857, -1.5446, -1.8238, -1,   #El último joint es abierto/cerrado
    -2.0857 , -1.3265, -2.0857, 0.0349, -1.8238, -1,
    -ca.inf,-ca.inf,-ca.inf,-ca.inf,-ca.inf,-ca.inf]   #El último joint es abierto/cerrado

q_max = [2.086017, 0.200015, #Chequear Head-Yaw/Pitch
    0.740810, 0.790477, 0.484090, 2.112528, 0.922747, 0.769001, #Chequear ankle-pitch/roll
    0.740810, 0.484090, 2.120198, 2.112528, 0.932056, 0.397935, #Chequear ankle-pitch/roll
    2.0857 , 1.3265, 2.0857, -0.0349, 1.8238, 1,  #El último joint es abierto/cerrado
    2.0857 , 0.3142, 2.0857, 1.5446, 1.8238, 1,
    ca.inf,ca.inf,ca.inf,ca.inf,ca.inf,ca.inf]   #El último joint es abierto/cerrado

#dq_min=[-2.085, -2.085]
dq_max = [8.26797, 7.19047,
        4.16174,4.16174,6.40239,6.40239,6.40239,4.16174,
        4.16174,4.16174,6.40239,6.40239,6.40239,4.16174,
        8.26797,7.19407,8.26797,7.19407,24.6229, 8.33,#last speed is from hand (which is none)
        8.26797,7.19407,8.26797,7.19407,24.6229, 8.33,
        ca.inf,ca.inf,ca.inf,ca.inf,ca.inf,ca.inf] #last speed is from hand (which is none)
dq_min=[-x for x in dq_max]

"""tau_max = [0.0143,  0.0143, 
           0.065, 0.065, 0.065, 0.065, 0.065, 0.065,  
            0.065, 0.065, 0.065, 0.065, 0.065, 0.065,  
            0.0143, 0.0143, 0.0143, 0.0143, 0.0143, 0.0094, 
            0.0143,  0.0143, 0.0143, 0.0143, 0.0143, 0.0094
]

tau_min =[-x for x in tau_max]"""

# Torque nominal por tipo de motor (Nm)
torque_nominal = {1: 0.0161, 2: 0.0049, 3: 0.0062}

# Lista de articulaciones con su reducción y tipo de motor
joint_info = [
    ("HeadYaw", 150.27, 3), ("HeadPitch", 173.22, 3),
    ("LHipYawPitch", 201.3, 1), ("LHipRoll", 201.3, 1), ("LHipPitch", 130.85, 1),
    ("LKneePitch", 130.85, 1), ("LAnklePitch", 130.85, 1), ("LAnkleRoll", 201.3, 1),
    ("RHipYawPitch", 201.3, 1), ("RHipRoll", 201.3, 1), ("RHipPitch", 130.85, 1),
    ("RKneePitch", 130.85, 1), ("RAnklePitch", 130.85, 1), ("RAnkleRoll", 201.3, 1),
    ("LShoulderPitch", 150.27, 3), ("LShoulderRoll", 173.22, 3), ("LElbowYaw", 150.27, 3),
    ("LElbowRoll", 173.22, 3), ("LWristYaw", 50.61, 2), ("LHand", 36.24, 2),
    ("RShoulderPitch", 150.27, 3), ("RShoulderRoll", 173.22, 3), ("RElbowYaw", 150.27, 3),
    ("RElbowRoll", 173.22, 3), ("RWristYaw", 50.61, 2), ("RHand", 36.24, 2)
]

# Cálculo del torque máximo por articulación (en Nm)
tau_max = [torque_nominal[motor_type] * reduction for _, reduction, motor_type in joint_info]


tau_min =[-x for x in tau_max]