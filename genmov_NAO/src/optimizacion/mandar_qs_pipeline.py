import qi
import time
import numpy as np
from math import atan2, sqrt
from read_data import *
import threading
import sys

#dir = sys.argv[1]
dir = "caso2"

csv='/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/motions/{dir}.csv'.format(dir=dir)
reconstructed_data, left_contact, right_contact =reconstruct(csv)

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


    
def compute_pitch_error(com, ankle_L, ankle_R):
 
    # Punto medio entre ambos tobillos
    midpoint = 0.5 * (np.array(ankle_L) + np.array(ankle_R))
    
    # Centro deseado del polígono de soporte (avanzado en X)
    support_center = midpoint + np.array([0.015, 0.0, 0.0])

    # Vector del support_center al CoM
    v = np.array(com) - support_center

    # Ángulo de inclinación real (CoM respecto al support_center) en XZ
    theta_com_pitch = atan2(v[2], v[0])

    # Ángulo deseado: CoM perfectamente sobre support_center => vertical => 0 rad
    theta_sup_pitch = 0.0

    pitch_error = theta_sup_pitch - theta_com_pitch

    return pitch_error

pid_pitch = PID(Kp=0.816, Ti=0.5, Td=0.125)
pid_roll  = PID(Kp=0.888, Ti=0.35, Td=0.875)
                
dt_frame = 0.05      # frecuencia de trayectoria (20 Hz)
#dt_pid   = 0.01      # frecuencia del PID (100 Hz)

com_data=np.zeros((2,len(right_contact)))

#def pid_balance_loop(session, i_func):#, q_func):
    # motion = session.service("ALMotion")
    # last_support_leg = None
    # print("Fuera del while, dentro del pid")
    
    # while True:
    #     global com_data
    #     i = i_func()
    #     #q_i = q_func()

    #     if left_contact[i] == 1 and right_contact[i] == 0:
    #         current_support = "left"
    #         com = motion.getCOM("Body", 2, True)
    #         com_data[:,i]=com[0:2]
    #         ankle = np.array(motion.getPosition("LLeg", 2, True)[0:3])
    #         support = ankle + np.array([0.015, 0.00, 0])
    #         # pitch_index = 6
    #         # roll_index = 7

    #     elif right_contact[i] == 1 and left_contact[i] == 0:
    #         current_support = "right"
    #         com = motion.getCOM("Body", 2, True)
    #         com_data[:,i]=com[0:2]
    #         ankle = np.array(motion.getPosition("RLeg", 2, True)[0:3])
    #         support = ankle + np.array([0.015, 0.00, 0])
    #         # pitch_index = 12
    #         # roll_index = 13

    #     else:
    #         current_support = None
    #         time.sleep(dt_pid)
    #         com = motion.getCOM("Body", 2, True)
    #         com_data[:,i]=com[0:2]
    #         continue


        

    #     if current_support != last_support_leg:
    #         pid_pitch.reset()
    #         pid_roll.reset()
    #         print(f"Cambio de pie de soporte: {last_support_leg} → {current_support}")
    #         last_support_leg = current_support

    #     # Solo aplicar corrección si hay un único pie de soporte
    #     if current_support is not None:
    #         print("CORRIGIENDO")
    #         pitch_error, roll_error = compute_pitch_roll_error(com, ankle, support)

    #         ankle_names = ["LAnklePitch", "LAnkleRoll", "RAnklePitch", "RAnkleRoll"]
    #         q_ankles = motion.getAngles(ankle_names, True)

    #         if current_support == "left":
    #             q_ankles[0] += 0.8*pitch_error#pid_pitch.compute(pitch_error, dt_pid)  # LAnklePitch
    #             q_ankles[1] += 0.8*roll_error#pid_roll.compute(roll_error, dt_pid)    # LAnkleRoll
    #             motion.setAngles(ankle_names[:2], q_ankles[:2], 0.3)

    #         elif current_support == "right":
    #             q_ankles[2] += pid_pitch.compute(pitch_error, dt_pid)  # RAnklePitch
    #             q_ankles[3] += pid_roll.compute(roll_error, dt_pid)    # RAnkleRoll
    #             motion.setAngles(ankle_names[2:], q_ankles[2:], 0.3)

    #     time.sleep(dt_pid)


last_support_leg = None
com_data=np.zeros((2,len(right_contact)))

def joints_all(session, q,i,inicial=False):
    global last_support_leg

    motion = session.service("ALMotion")

    q_pid = q.copy()

    if len(q) != len(joint_names):
        print("Error: q must have 26 values corresponding to all the joints.")
        return

    start_time = time.perf_counter()

    if inicial:
        motion.angleInterpolationWithSpeed(joint_names, q, 0.4)
    else:
        
        if left_contact[i] == 1 and right_contact[i] == 0:
            current_support = "left"
            com = motion.getCOM("Body", 2, True)
            #com_data[:,i]=com[0:2] # guarar data
            ankle = np.array(motion.getPosition("LLeg", 2, True)[0:3])
            support = ankle + np.array([0.015, 0.01, 0])
            # pitch_index = 6
            # roll_index = 7

        elif right_contact[i] == 1 and left_contact[i] == 0:
            current_support = "right"
            com = motion.getCOM("Body", 2, True)
            #com_data[:,i]=com[0:2]
            ankle = np.array(motion.getPosition("RLeg", 2, True)[0:3])
            support = ankle + np.array([0.015,-0.01, 0])
            #pitch_index = 12
            #roll_index = 13
        
        else:
            current_support = None
            motion.setAngles(joint_names, q, 0.6)
            # com = motion.getCOM("Body", 0, True)
            # #r_ankle = np.array(motion.getPosition("RLeg", 2, True)[0:3])
            # l_ankle = np.array(motion.getPosition("LLeg", 0, True)[0:3])
            # support = l_ankle + np.array([0.015,-0.02, 0])
            # pitch_error, roll_error = compute_pitch_roll_error(com, l_ankle, support) #compute_pitch_error(com,l_ankle,r_ankle)
            # q_pid[6]   += 0.8*pitch_error
            # q_pid[12]  += 0.8*pitch_error
 
            # q_pid = [float(x) for x in q_pid]
            # motion.setAngles(joint_names, q_pid, 0.8)
            while (time.perf_counter() - start_time) < 0.065: #0.075
                time.sleep(0.001) 
                # salir de la funcion
            return

        if current_support != last_support_leg:
            pid_pitch.reset()
            pid_roll.reset()
            print(f"Cambio de pie de soporte: {last_support_leg} → {current_support}")
            last_support_leg = current_support

        # Solo aplicar corrección si hay un único pie de soporte
        if current_support is not None:
            print("CORRIGIENDO")
            pitch_error, roll_error = compute_pitch_roll_error(com, ankle, support)

            #ankle_names = ["LAnklePitch", "LAnkleRoll", "RAnklePitch", "RAnkleRoll"]
            #q_ankles = motion.getAngles(ankle_names, True)

            if current_support == "left":
                q_pid[6] += 0.5*pitch_error#pid_pitch.compute(pitch_error, dt_pid)  # LAnklePitch
                q_pid[7] += 0.5*roll_error#pid_roll.compute(roll_error, dt_pid)    # LAnkleRoll

            elif current_support == "right":
                q_pid[12] += 0.5*pitch_error#pid_pitch.compute(pitch_error, dt_frame)  # RAnklePitch
                q_pid[13] += 0.5*roll_error#pid_roll.compute(roll_error, dt_frame)    # RAnkleRoll

        q_pid = [float(x) for x in q_pid]
        motion.setAngles(joint_names, q_pid, 0.6)
        # Esperar hasta completar 0.05 segundos
        while (time.perf_counter() - start_time) < 0.065: #0.075
            time.sleep(0.001)  


def main():
    safe=input("¿Desea mandar el movimiento al robot? (y/n)\n *si no se pone y se considerará que no se envía el movimiento:")
    is_safe = safe=="y"
    if not is_safe: 
        exit()
    else:
        t_init1=time.time()
        data = np.load(f"/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/output_trajectory/{dir}/q.npz")
        q_full = data["q_full"]
        q_full = np.array(q_full)

        #robot_ip = sys.argv[2]
        robot_ip="169.254.199.108"
        robot_port = 9559
        max_retries = 10
        retry_delay = 1

        session = qi.Session()
        for attempt in range(1, max_retries + 1):
            try:
                print(f"Attempt {attempt} to connect to {robot_ip}:{robot_port}")
                session.connect(f"tcp://{robot_ip}:{robot_port}")
                print("Successfully connected to the robot.")
                break
            except Exception as e:
                print(f"Connection failed: {e}")
                if attempt == max_retries:
                    print("Max retries reached. Exiting.")
                    sys.exit(1)
                time.sleep(retry_delay)
                retry_delay *= 1.2
        
        t_fin1=time.time()
        filename=f'/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/times/{dir}.txt'
        with open(filename,'r') as file:
            lines = file.readlines()
        while len(lines) < 4:
            lines.append("\n")
        lines[3]=f"{t_fin1-t_init1}\n"

        with open(filename, 'w') as file:
            file.writelines(lines)
        print("Written to line 3")

        posture=session.service("ALRobotPosture")

        q0 = (q_full[6:, 0].flatten())

        # global i_actual
        # i_actual = 0
        # global q_actual 
        # q_actual = q0

        # Get required services
        motion = session.service("ALMotion")

        print(f"Initial joint angles: \n {motion.getAngles(joint_names,True)}") #True to get actual sensor balue

        motion.stiffnessInterpolation("Head", 1.0, 1.0)
        motion.stiffnessInterpolation("LArm", 1.0, 1.0)
        motion.stiffnessInterpolation("RArm", 1.0, 1.0)
        motion.stiffnessInterpolation("LLeg", 1.0, 1.0)
        motion.stiffnessInterpolation("RLeg", 1.0, 1.0)

        posture.goToPosture("StandInit", 0.6)
        print("Poniendose de pie")
        time.sleep(3)

        # Lanzar hilo de control PID en paralelo
        # threading.Thread(
        #     target=pid_balance_loop,
        #     args=(session, lambda: i_actual),#, lambda: q_actual),
        #     daemon=True
        # ).start()

        # Ahora mandamos los qs

        origin_com2 = np.array(motion.getCOM("Body", 2, True)[0:2])
        #origin_com2 = np.array([-com[1], com[0]])

        origin_com1 = np.array(motion.getCOM("Body", 1, True)[0:2])

        dif = origin_com2 - origin_com1

        joints_all(session, q0.tolist(),0,True)
        

        for i in range(len(q_full[0,:])):
            #i_actual = i
            # Enviar el resto de la trayectoria
            q_i = q_full[6:, i].flatten()
            #q_actual = q_i
            start_time = time.time()
            joints_all(session,q_i.tolist(),i)
            com = motion.getCOM("Body", 1, True)
            com_data[:, i] = np.array(com[0:2]) + dif#- origin_com[0:2]
            elapsed_time = time.time() - start_time
            print(f"Step {i}: {elapsed_time:.4f} seconds")


        time.sleep(2)
        # Close the Qi session properly
        print("Closing Qi session...")
        np.savez(f'/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/com/{dir}.npz', com_data=com_data)
        session.close()
        t_fin2=time.time()

        filename=f'/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/times/{dir}.txt'
        with open(filename,'r') as file:
            lines = file.readlines()
        while len(lines) < 5:
            lines.append("\n")
        lines[4]=f"{t_fin2-t_fin1}\n"
        
        
        with open(filename, 'w') as file:
            file.writelines(lines)
        print("Written to line 4")

        

if __name__ == "__main__":
    main()