import qi
import time
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R
from os.path import join
from forward_kinematics import *
from read_data import *
# Cargar modelo de NAO con base flotante

model_path = "/home/invitado8/proy_ws/src/nao/nao_utec/urdf/"
urdf_filename = "naoV5blue.urdf"
urdf_model_path = join(model_path, urdf_filename)
model = pin.buildModelFromUrdf(urdf_model_path, pin.JointModelFreeFlyer())
reconstructed_data, left_contact, right_contact =reconstruct("Step2.csv")


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.e_sum = 0
        self.e_last = 0

    def compute(self, e, dt):
        de = (e - self.e_last) / dt
        self.e_sum += e * dt
        self.e_last = e
        return self.Kp * e + self.Ki * self.e_sum + self.Kd * de
    
def center_of_mass_numpy(q_rpy):
    xyz = q_rpy[0:3]
    rpy = q_rpy[3:6]
    q_act = q_rpy[6:]

    quat_xyzw  = R.from_euler('zyx', [rpy[2],rpy[1],rpy[0]]).as_quat()
    quat = np.array([quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]])
    q_pin = np.concatenate([xyz, quat, q_act])

    data = model.createData()
    com = model.computeCenterOfMass(data, q_pin)
    return com


pid_pitch = PID(Kp=0.816, Ki=1.632, Kd=0.102)
pid_roll  = PID(Kp=0.888, Ki=2.537, Kd=0.777)
dt_frame = 0.05      # frecuencia de trayectoria (20 Hz)
dt_pid   = 0.01      # frecuencia del PID (100 Hz)


def joints_all(session, q, inicial=False):
    import time
    import numpy as np

    motion = session.service("ALMotion")
    memory = session.service("ALMemory")

    joint_names = [
        "HeadYaw", "HeadPitch",
        "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
        "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"
    ]

    if len(q) != len(joint_names):
        print("Error: q must have 26 values corresponding to all the joints.")
        return

    start_time = time.perf_counter()

    if inicial:
        motion.angleInterpolationWithSpeed(joint_names, q, 0.25)
    else:
        dt_pid = 0.01  # 100 Hz

        while (time.perf_counter() - start_time) < 0.0485:
            q_pid = q.copy()

            # Solo si hay un pie en contacto
            angle_x = memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
            angle_y = memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
            angle_z = memory.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value")
            q_base = [0.0, 0.0, 0.0, angle_x, angle_y, -angle_z]
            q_full = np.concatenate([q_base, q_pid])

            if left_contact == 1 and right_contact == 0:
                com = center_of_mass_numpy(q_full)[:2]
                foot = fkine_left_foot_constant(q_pid[2:8], q_base, "c")[0:2,3]
                error = com - foot
                q_pid[6] += -pid_pitch.compute(error[0], dt_pid)  # LAnklePitch
                q_pid[7] += -pid_roll.compute(error[1], dt_pid)   # LAnkleRoll

            elif right_contact == 1 and left_contact == 0:
                com = center_of_mass_numpy(q_full)[:2]
                foot = fkine_right_foot_constant(q_pid[8:14], q_base, "c")[0:2,3]
                error = com - foot
                q_pid[12] += -pid_pitch.compute(error[0], dt_pid)  # RAnklePitch
                q_pid[13] += -pid_roll.compute(error[1], dt_pid)   # RAnkleRoll

            motion.angleInterpolationWithSpeed(joint_names, q_pid, 0.75)
            time.sleep(dt_pid)



def main():
    data = np.load("warmstart_brazos.npz")
    q_full = data["q_full"]
    q_full = np.array(q_full)

    robot_ip = "192.168.10.101"
    robot_port = 9559

    # Create an application instance
    app = qi.Application(["NAOqiApp", f"--qi-url=tcp://{robot_ip}:{robot_port}"])
    app.start()  # Start the application
    session = app.session  # Get the session from the application
    posture=session.service("ALRobotPosture")
    all_joint_names=[
            "HeadYaw","HeadPitch",
            "LHipYawPitch","LHipRoll","LHipPitch","LKneePitch","LAnklePitch","LAnkleRoll",
            "RHipYawPitch","RHipRoll","RHipPitch","RKneePitch","RAnklePitch","RAnkleRoll",
            "LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw","LHand",
            "RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw","RHand"
            ]

    try:
        print("Connected to the robot!")

        # Get required services
        motion = session.service("ALMotion")
    
        print(f"Initial joint angles: \n {motion.getAngles(all_joint_names,True)}") #True to get actual sensor balue

        motion.stiffnessInterpolation("Head", 1.0, 1.0)
        motion.stiffnessInterpolation("LArm", 1.0, 1.0)
        motion.stiffnessInterpolation("RArm", 1.0, 1.0)
        motion.stiffnessInterpolation("LLeg", 1.0, 1.0)
        motion.stiffnessInterpolation("RLeg", 1.0, 1.0)

        posture.goToPosture("StandInit", 0.5)

        print("Poniendose de pie")

        time.sleep(3)
        # Ahora mandamos los qs
        q0 = np.array(q_full[6:, 0].flatten())
        joints_all(session, q0.tolist(),True)



        for i in range(len(q_full[0,:])):
             # Enviar el resto de la trayectoria
            q_start_frame = q_full[6:, i].flatten()
        
            start_time = time.time()
            joints_all(session,q_start_frame.tolist())
            elapsed_time = time.time() - start_time


            print(f"Step {i}: {elapsed_time:.4f} seconds")
            
        
            print(i)


    except Exception as e:
        print(f"Failed to connect to robot: {e}")


    finally:
        # Close the Qi session properly
        #np.savez("angles.npz",angles_list = angles_list)
        print("Closing Qi session...")
        session.close()


if __name__ == "__main__":
    main()