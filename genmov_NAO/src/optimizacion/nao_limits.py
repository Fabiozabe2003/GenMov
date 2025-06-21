import numpy as np
import casadi as ca



# Limits
# http://doc.aldebaran.com/2-1/family/robots/joints_robot.html
q_min = [-ca.inf,-ca.inf, -ca.inf , -ca.inf, -ca.inf, -ca.inf,
    -ca.pi/3, -0.330041, #Chequear Head-Yaw/Pitch
    -1.14529, -0.379435, -1.53589, -0.0923279, -1.18944, -0.397761,
    -1.14529, -0.79046, -1.53589, -0.0923279, -1.1863, -0.768992, 
    -2.08567, -0.314159, -2.08567, -1.54462, -1.82387, 0.0, 
    -2.08567, -1.32645, -2.08567, 0.0349066, -1.82387, 0.0]


q_max = [ ca.inf,ca.inf, ca.inf , ca.inf, ca.inf, ca.inf,
      ca.pi/3, 0.200015, 
      0.740718, 0.79046, 0.48398, 2.11255, 0.922581, 0.768992,
      0.740718, 0.379435, 0.48398, 2.11255, 0.932006, 0.397761, 
      2.08567, 1.32645, 2.08567, -0.0349066, 1.82387, 1.0,
      2.08567, 0.314159, 2.08567, 1.54462, 1.82387, 1.0]


dq_max = [8.26797, 7.19047,
          4.16174,4.16174,6.40239,6.40239,6.40239,4.16174,
          4.16174,4.16174,6.40239,6.40239,6.40239,4.16174,
          8.26797,7.19407,8.26797,7.19407,24.6229, 8.33, 
          8.26797,7.19407,8.26797,7.19407,24.6229, 8.33 #last speed is from hand (which is none)
        ]

dq_max = [x/9 for x in dq_max]

dq_min=[-x for x in dq_max]


# http://doc.aldebaran.com/2-1/family/robots/motors_robot.html


# Torque nominal por tipo de motor (Nm)
torque_nominal =  {1: 0.0161, 2: 0.0049, 3: 0.0062} # Le he quitado un 0 a los decimales para prueba
#torque_nominal = {1: 0.0680, 2: 0.0094, 3:0.0143}

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


tau_max = [1.547, 1.532, 
           3.348, 3.348, 3.023, 3.023, 3.023, 3.348, 
           3.348, 3.348, 3.0226, 3.0226, 3.0226, 3.348, 
           1.329, 1.7835, 1.547, 1.532, 0.4075, 0.292,
             1.329, 1.783, 1.547, 1.532, 0.4075, 0.292]

tau_max = [x*0.9 for x in tau_max]

tau_min =[-x for x in tau_max]
