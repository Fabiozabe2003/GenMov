import qi
import time
import numpy as np
from math import atan2, sqrt
from read_data import *
import threading

reconstructed_data, left_contact, right_contact =reconstruct("Step2.csv")

joint_names = [
        "HeadYaw", "HeadPitch",
        "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
        "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"
    ]

class Filter:
    def __init__(self, alpha=1.0, initval=None):
        self.val = initval
        self.alpha = alpha
        self.ralpha = 1.0 - alpha

    def set(self, val):
        if self.val is None:
            self.val = val
        else:
            self.val = self.alpha * val + self.ralpha * self.val
        return self.val

    def reset(self, val):
        self.val = val

    def get(self):
        return self.val


class PID:
    def __init__(self, Kp, Ti, Td, alpha=0.3, u_limit=0.3, d_limit=5.0):
        self.Kp = Kp
        self.Ki = Kp / Ti if Ti != 0 else 0.0
        self.Kd = Kp * Td
        self.e_sum = 0.0
        self.e_last = None
        self.u_limit = u_limit
        self.d_limit = d_limit
        self.filter = Filter(alpha)

    def reset(self):
        self.e_sum = 0.0
        self.e_last = None
        self.filter.reset(0.0)

    def compute(self, e, dt):
        e_filtered = self.filter.set(e)

        if self.e_last is None:
            self.e_last = e_filtered

        de = (e_filtered - self.e_last) / dt
        de = max(min(de, self.d_limit), -self.d_limit)

        self.e_sum += e_filtered * dt
        self.e_last = e_filtered

        u = self.Kp * e_filtered + self.Ki * self.e_sum + self.Kd * de
        u = max(min(u, self.u_limit), -self.u_limit)
        return u
    
def compute_pitch_roll_error(com, ankle, support_center):
    v = com - ankle
    theta_com_pitch = atan2(v[2], v[0])
    theta_sup_pitch = atan2(sqrt(v[2]**2 + v[0]**2), support_center[0] - ankle[0])
    pitch_error = theta_sup_pitch - theta_com_pitch

    theta_com_roll = atan2(v[2], v[1])
    theta_sup_roll = atan2(sqrt(v[2]**2 + v[1]**2), support_center[1] - ankle[1])
    roll_error = theta_com_roll - theta_sup_roll
    return pitch_error, roll_error

pid_pitch = PID(Kp=0.816, Ti=0.5, Td=0.125)
pid_roll  = PID(Kp=0.888, Ti=0.35, Td=0.875)
                
                
dt_frame = 0.05      # frecuencia de trayectoria (20 Hz)
dt_pid   = 0.01      # frecuencia del PID (100 Hz)


def joints_all(session, q, i, inicial=False):
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
        print(len(q))
    else:
        dt_pid = 0.01 # 100 Hz

        while (time.perf_counter() - start_time) < 0.0485:
            q_pid = q.copy()
            # Solo si hay un pie en contacto
          
            if left_contact[i] == 1 and right_contact[i] == 0:
                com = motion.getCOM("Body", 2, True) 
                ankle_l = np.array(motion.getPosition("LLeg",2,True)[0:3])
                support_l = ankle_l + np.array([0.015, 0.01, 0])

                pitch_error, roll_error = compute_pitch_roll_error(com, ankle_l, support_l)
                # print("roll_error",roll_error)
                # print("AnklROll antes:", q_pid[7])

                q_pid[6] += pitch_error#pid_pitch.compute(pitch_error, dt_pid)  # LAnklePitch
                q_pid[7] += roll_error#pid_roll.compute(roll_error, dt_pid)  # LAnkleRoll

                
                # print("AnklROll:", q_pid[7])

            elif right_contact[i] == 1 and left_contact[i] == 0:
                com = motion.getCOM("Body", 2, True) 
                ankle_r= np.array(motion.getPosition("RLeg",2,True)[0:3])
                support_r = ankle_r + np.array([0.015, -0.01, 0])

                pitch_error, roll_error = compute_pitch_roll_error(com, ankle_r, support_r)
            
                q_pid[12] += pid_pitch.compute(pitch_error, dt_pid)  # RAnklePitch
                q_pid[13] += pid_roll.compute(roll_error, dt_pid)   # RAnkleRoll

            q_pid = [float(x) for x in q_pid]
            motion.setAngles(joint_names, q_pid, 0.7)
            #time.sleep(dt_pid)        



def main():
    data = np.load("warmstart_brazos.npz")
    q_full = data["q_full"]
    q_full = np.array(q_full)

    robot_ip = "169.254.38.159"
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
        joints_all(session, q0.tolist(), 1 ,True)



        for i in range(len(q_full[0,:])):
             # Enviar el resto de la trayectoria
            q_start_frame = q_full[6:, i].flatten()
        
            start_time = time.time()
            joints_all(session,q_start_frame.tolist(),i)
            elapsed_time = time.time() - start_time


            print(f"Step {i}: {elapsed_time:.4f} seconds")
            
        
             # print(i)


    except Exception as e:
        print(f"Failed to connect to robot: {e}")


    finally:
        # Close the Qi session properly
        #np.savez("angles.npz",angles_list = angles_list)
        print("Closing Qi session...")
        session.close()


if __name__ == "__main__":
    main()