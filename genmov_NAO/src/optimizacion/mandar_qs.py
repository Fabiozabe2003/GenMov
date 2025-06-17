import qi
import time
import numpy as np

def get_imu(session):
    memory = session.service("ALMemory")

    # Leer orientación (ángulos)
    angle_x = memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
    angle_y = memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
    angle_z = memory.getData("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value")

    # Imprimir en consola
    #print(f"Ángulos → x: {angle_x:.3f}, y: {angle_y:.3f}, z: {angle_z:.3f}")
    return angle_x, angle_y, angle_z


def joints_all(session, q, inicial=False):
    motion = session.service("ALMotion")

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

    time_list=[0.05]*26

    if inicial:
        motion.angleInterpolationWithSpeed(joint_names, q, 0.25)

    else:
        # Ejecutar en modo asíncrono (no bloqueante)
        motion.angleInterpolationWithSpeed(joint_names, q, 1.0) #0.75 para 0.05

        # Esperar hasta 0.05 s como máximo
        #while (time.perf_counter() - start_time) < 0.0485:
        #     time.sleep(0.001)  # espera breve para no sobrecargar CPU

    # Si la ejecución llega aquí, han pasado 0.05 s o menos
    


def execute_bezier_trajectory(session, q_full):  
    motion = session.service("ALMotion")
    
    # Joint names: NAO's actuated joint order
    joint_names = [
        "HeadYaw", "HeadPitch",
        "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
        "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"
    ]

    q = q_full[6:, :]  # Skip floating base (6 first rows)
    N = q.shape[1]
    dt = 0.05 # 50 ms per step

    names = []
    times = []
    keys = []

    for i, joint in enumerate(joint_names):
        names.append(joint)
        joint_times = []
        joint_keys = []

        for k in range(N):
            t = dt * k
            val = float(q[i, k])

            # Smooth Bezier using default tangent (can be tuned)
            if k == 0:
                # First keyframe: zero incoming tangent
                key = [val, [3, -dt, 0.0], [3, dt, 0.0]]
            elif k == N - 1:
                # Last keyframe: zero outgoing tangent
                key = [val, [3, -dt, 0.0], [3, 0.0, 0.0]]
            else:
                key = [val, [3, -dt, 0.0], [3, dt, 0.0]]

            joint_times.append(t)
            joint_keys.append(key)

        times.append(joint_times)
        keys.append(joint_keys)

    # Send trajectory
    print("Ejecutando bezier:")
    motion.angleInterpolationBezier(names, times, keys)



def main():
    data = np.load("warmstart_brazos.npz")
    q_full = data["q_full"]
    q_full = np.array(q_full)

    # data = np.load("q_smooth.npz")
    # q_full = data["q_smooth"]
    # q_full = np.array(q_full)

    robot_ip = "192.168.10.104"
    #robot_ip = "169.254.38.159"# azul (main character)
    # robot_ip = "169.254.199.108" #azul español (cinde)
    #robot_ip = "169.254.129.144" # naranja
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

        time.sleep(3)

        execute_bezier_trajectory(session,q_full)

        # for i in range(len(q_full[0,:])):
        #      # Enviar el resto de la trayectoria
        #     q_start_frame = q_full[6:, i].flatten()
        #     #q_end_frame = q_full[6:, i+1].flatten()

        #     #print("Velocidad requerida por joint:", (q_start_frame-q_end_frame)/0.05)

        #     # qs = q_full[6:32, i]  # Tomar los últimos 26 DoF del instante i
        #     # qs1 = q_full[6:32,i+1]
        #     # dqs = (qs1 - qs) / 0.05
        #     # dq = float(np.abs(dqs) / dq_max)
        
        #     start_time = time.time()
        #     joints_all(session,q_start_frame.tolist())
        #     #joints_all_bezier_segment(session, q_start_frame.tolist(), q_end_frame.tolist(), 0.05)
        #     elapsed_time = time.time() - start_time


        #     print(f"Step {i}: {elapsed_time:.4f} seconds")
            
        #     #angle_x,angle_y,angle_z=get_imu(session)
        #     #angles_list.append([angle_x,angle_y,angle_z])
        #     #time.sleep(0.075)
        #     print(i)


    except Exception as e:
        print(f"Failed to connect to robot: {e}")


    finally:
        # Close the Qi session properly
        #np.savez("angles.npz",angles_list = angles_list)
        print("Closing Qi session...")
        session.close()


if __name__ == "__main__":
    main()