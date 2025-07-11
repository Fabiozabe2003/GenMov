#!/usr/bin/env python3
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import qi
import rospy
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import tf2_ros
from markers import *
from forward_kinematics import *


joint_indices = {
    "l_arm": slice(14 + 6, 20 + 6),
    "r_arm": slice(20 + 6 , 26 + 6),
    "f_base": slice(0,6),
    "l_leg": slice(2  + 6, 8 + 6),
    "r_leg": slice(8 + 6, 14 +6),
}



# IP y puerto del NAO
robot_ip = "169.254.199.108"
robot_port = 9559

# Lista de nombres de articulaciones según URDF
#jaja así es
jnames = [
    "HeadYaw", "HeadPitch",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand",
    "RFinger23", "RFinger13", "RFinger12",
    "LFinger21", "LFinger13", "LFinger11",
    "RFinger22", "LFinger22", "RFinger21",
    "LFinger12", "RFinger11", "LFinger23",
    "LThumb1", "RThumb1", "RThumb2", "LThumb2"
]

def publish_tf(br, position, rpy, frame_id="world", child_frame_id="base_link"):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = position[0]
    t.transform.translation.y = position[1]
    t.transform.translation.z = position[2]

    quat = R.from_euler('zyx', [rpy[2], rpy[1], rpy[0]]).as_quat()
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    br.sendTransform(t)


import pinocchio as pin
from os.path import join
# Cargar modelo de NAO con base flotante

model_path = "/home/invitado8/proy_ws/src/nao/nao_utec/urdf/"
urdf_filename = "naoV40red.urdf"
urdf_model_path = join(model_path, urdf_filename)
model = pin.buildModelFromUrdf(urdf_model_path, pin.JointModelFreeFlyer())


def center_of_mass_numpy(q_rpy):
    xyz = q_rpy[0:3]
    rpy = q_rpy[3:6]
    q_act = q_rpy[6:]

    quat_xyzw = R.from_euler('zyx', [rpy[2],rpy[1],rpy[0]]).as_quat()
    quat = np.array([quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]])

    q_pin = np.concatenate([xyz, quat, q_act])

    data = model.createData()
    com =pin.centerOfMass(model, data, q_pin,False)
    #com = data.com[0]  

    return com


def main():
    rospy.init_node("nao_joint_publisher_qi", anonymous=True)
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(100)

    try:
        app = qi.Application(["NAOqiApp", f"--qi-url=tcp://{robot_ip}:{robot_port}"])
        app.start()
        session = app.session
        print("✅ Conectado al robot")
    except Exception as e:
        rospy.logerr(f"❌ Error al conectar con el robot: {e}")
        return

    motion = session.service("ALMotion")
    memory = session.service("ALMemory")

    bmarker_com = BallMarker(color['GREEN'])
    bmarker_ankle = BallMarker(color['RED'])
    bmarker_sup =  BallMarker(color['BLUE'])

    while not rospy.is_shutdown():
        try:
            q_actuated = motion.getAngles(jnames[:26], True)
            angle_x = memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
            angle_y = memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
            angle_z = -memory.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value")
        except Exception as e:
            rospy.logerr(f"❌ Error al leer datos del robot: {e}")
            continue
      

    
        # Calcular y mostrar el centro de masa
        com = motion.getCOM("Body", 2, True)  # FRAME_WORLD
        bmarker_com.xyz(com)


        ankle_l = np.array(motion.getPosition("LLeg",2,True)[0:3])
        support = ankle_l + np.array([0.015, 0.03, 0])
        #l_foot = fkine_left_foot_constant(q_full[joint_indices["l_leg"]],q_full[joint_indices["f_base"]],"c")[0:3,3]
        
        bmarker_ankle.xyz([ankle_l[0],ankle_l[1], ankle_l[2]])

        bmarker_sup.xyz(support)


        # ankle_r = np.array(motion.getPosition("RLeg",2,True)[0:3])
        # ankle_r = ankle_r + np.array([0.015, -0.01, 0])
        # bmarker_com.xyz([ankle_r[0],ankle_r[1], ankle_r[2]])


        # Publicar /joint_states
        jstate = JointState()
        jstate.header.stamp = rospy.Time.now()
        jstate.name = jnames
        q_extra = np.zeros(len(jnames) - 26)
        jstate.position = np.concatenate([q_actuated, q_extra]).tolist()
        pub.publish(jstate)
        print(jstate.position[joint_indices["r_leg"]])

        # Publicar TF de base_link
        base_position = np.array(motion.getPosition("Torso",2,True)[0:3])
        base_rpy = [angle_x, angle_y, angle_z]
        publish_tf(br, base_position, base_rpy)
        print()

        rate.sleep()

if __name__ == "__main__":
    main()
