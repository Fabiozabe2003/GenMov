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
        motion.angleInterpolationWithSpeed(joint_names, q, 0.1)

    else:
        # Ejecutar en modo asíncrono (no bloqueante)
        motion.angleInterpolationWithSpeed(joint_names, q, 0.75)

        # Esperar hasta 0.05 s como máximo
        while (time.perf_counter() - start_time) < 0.0485:
             time.sleep(0.001)  # espera breve para no sobrecargar CPU

    # Si la ejecución llega aquí, han pasado 0.05 s o menos
    



def joints_all_bezier_segment(session, q_start, q_end, time_step = 0.05):

    motion = session.service("ALMotion")

    joint_names = [
        "HeadYaw", "HeadPitch",
        "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
        "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
        "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
        "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"
    ]

    if len(q_start) != len(joint_names) or len(q_end) != len(joint_names):
        print(f"Error: q_start and q_end must have {len(joint_names)} values corresponding to all the joints.")
        return

  
    segment_times_for_all_joints = [[0.0, time_step] for _ in joint_names]

    handle_factor = 0.33 

    all_joint_control_points = []
    for j in range(len(joint_names)):
        angle_start = q_start[j]
        angle_end = q_end[j]

        # Control Point for the START of the segment (q_start)
        cp_start = [
            angle_start,
            # Handle1_start: [InterpolationType, dTime, dAngle]
            # No preceding curve influence if this is the effective start of a segment.
            # (dTime 0.0 means handle is at the control point in time)
            [2, 0.0, 0.0], 
            # Handle2_start: Defines the initial tangent for the curve *leaving* q_start.
            # dTime extends forward in time to pull the curve.
            [2, time_step * handle_factor, 0.0] 
        ]

        # Control Point for the END of the segment (q_end)
        cp_end = [
            angle_end,
            # Handle1_end: Defines the final tangent for the curve *arriving* at q_end.
            # dTime extends backward in time to pull the curve.
            [2, -time_step * handle_factor, 0.0], 
            # Handle2_end: No subsequent segment influence if this is the effective end.
            [2, 0.0, 0.0] 
        ]
        
        # Append the control points for the current joint.
        # It's a list containing the start and end control points for this joint.
        all_joint_control_points.append([cp_start, cp_end])

    try:
        # Execute the Bezier interpolation for this segment
        motion.angleInterpolationBezier(joint_names, segment_times_for_all_joints, all_joint_control_points)
        # print(f"Bezier interpolation completed for segment in {time_step:.2f}s.")
    except Exception as e:
        print(f"Error executing angleInterpolationBezier for segment: {e}")



def main():
    data = np.load("warmstart_brazos.npz")
    q_full = data["q_full"]
    q_full = np.array(q_full)

    robot_ip = "169.254.38.159"# azul (main character)
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

        # Ahora mandamos los qs
        q0 = np.array(q_full[6:, 0].flatten())
        joints_all(session, q0.tolist(),True)

        angles_list=[]
        time.sleep(3)


        dq_max = [8.26797, 7.19047,
                4.16174,4.16174,6.40239,6.40239,6.40239,4.16174,
                4.16174,4.16174,6.40239,6.40239,6.40239,4.16174,
                8.26797,7.19407,8.26797,7.19407,24.6229, 8.33,#last speed is from hand (which is none)
                8.26797,7.19407,8.26797,7.19407,24.6229, 8.33 #last speed is from hand (which is none)
                ]

        for i in range(92):
             # Enviar el resto de la trayectoria
            q_start_frame = q_full[6:, i].flatten()
            #q_end_frame = q_full[6:, i+1].flatten()

            #print("Velocidad requerida por joint:", (q_start_frame-q_end_frame)/0.05)

            # qs = q_full[6:32, i]  # Tomar los últimos 26 DoF del instante i
            # qs1 = q_full[6:32,i+1]
            # dqs = (qs1 - qs) / 0.05
            # dq = float(np.abs(dqs) / dq_max)
        
            start_time = time.time()
            joints_all(session,q_start_frame.tolist())
            #joints_all_bezier_segment(session, q_start_frame.tolist(), q_end_frame.tolist(), 0.05)
            elapsed_time = time.time() - start_time


            print(f"Step {i}: {elapsed_time:.4f} seconds")
            
            #angle_x,angle_y,angle_z=get_imu(session)
            #angles_list.append([angle_x,angle_y,angle_z])
            #time.sleep(0.075)
            print(i)


    except Exception as e:
        print(f"Failed to connect to robot: {e}")


    finally:
        # Close the Qi session properly
        np.savez("angles.npz",angles_list = angles_list)
        print("Closing Qi session...")
        session.close()


if __name__ == "__main__":
    main()