import numpy as np
#import cvxpy as cp
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
q_min = [-ca.inf,-ca.inf, -ca.inf , -ca.inf, -ca.inf, -ca.inf,
    -2.086017, -0.330041, #Chequear Head-Yaw/Pitch
    -1.145303, -0.379472, -1.535889, -0.092346, -1.189516, -0.397880, #Chequear ankle-pitch/roll
    -1.145303, -0.790447, -1.535889, -0.103083, -1.186448, -0.768992, #Chequear ankle-pitch/roll
    -2.0857 , -0.3142, -2.0857, -1.5446, -1.8238, -1,   #El último joint es abierto/cerrado
    -2.0857 , -1.3265, -2.0857, 0.0349, -1.8238, -1
     ]   #El último joint es abierto/cerrado

q_max = [ ca.inf,ca.inf, ca.inf , ca.inf, ca.inf, ca.inf,
    2.086017, 0.200015, #Chequear Head-Yaw/Pitch
    0.740810, 0.790477, 0.484090, 2.112528, 0.922747, 0.769001, #Chequear ankle-pitch/roll
    0.740810, 0.484090, 2.120198, 2.112528, 0.932056, 0.397935, #Chequear ankle-pitch/roll
    2.0857 , 1.3265, 2.0857, -0.0349, 1.8238, 1,  #El último joint es abierto/cerrado
    2.0857 , 0.3142, 2.0857, 1.5446, 1.8238, 1
      ]   #El último joint es abierto/cerrado


dq_max=[2.085, 2.085,
        1.145,1.145,1.145,2.085,1.145,1.145,
        1.145,1.145,1.145,2.085,1.145,1.145,
        2.085,2.085,4.161,4.161,6.283, 2.085,#last speed is from hand (which is none)
        2.085,2.085,4.161,4.161,6.283, 2.085#last speed is from hand (which is none)
        ]

dq_min=[-x for x in dq_max]


# http://doc.aldebaran.com/2-1/family/robots/motors_robot.html

tau_max = [0.0143,  0.0143, 
           0.065, 0.065, 0.065, 0.065, 0.065, 0.065,  
            0.065, 0.065, 0.065, 0.065, 0.065, 0.065,  
            0.0143, 0.0143, 0.0143, 0.0143, 0.0143, 0.0094, 
            0.0143,  0.0143, 0.0143, 0.0143, 0.0143, 0.0094
]

tau_max = [1000*x for x in tau_max]

tau_min =[-x for x in tau_max]
