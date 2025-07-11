import pybullet as p
from qibullet import SimulationManager
import time
import numpy as np
from nao_limits import *
import sys
#dir="caso3"
dir=sys.argv[1]
q_inicial_deseado = [0.04597806930541992, 0.28067994117736816, 
                     0.065986785888671875, -0.08432793617248535, 0.25928807258605957, -0.09208202362060547,
                      0.11654210090637207, 0.07674193382263184, 0.065986785888671875, 0.06447005271911621, 
                      0.26840806007385254, -0.09232791513204575, 0.09514999389648438, -0.010695934295654297, 
                      1.2839161157608032, -0.06293606758117676, -1.4373998641967773, -0.5736739635467529, 
                      1.8238691091537476, 0.08920001983642578, 1.2441158294677734, 0.029103994369506836,
                        1.254770040512085, 0.48018407821655273, -0.6550600528717041, 0.3999999761581421]


if __name__ == "__main__":
    # Iniciar el manejador de simulación
    simulation_manager = SimulationManager()
    
    # Lanzar simulación con GUI
    client = simulation_manager.launchSimulation(gui=True)

    p.resetDebugVisualizerCamera(1.0,90,0.0, [0,0,0.4])
    # Spawn del robot NAO
    robot = simulation_manager.spawnNao(
        client,
        translation=[0, 0, 0],
        quaternion=[0, 0, 0, 1],
        spawn_ground_plane=True
    )

    robot_id = 1
    mu = 0.8
    #p.changeDynamics(0, -1, lateralFriction=mu)
    #p.changeDynamics(robot_id, robot.joint_dict["RAnkleRoll"].index, lateralFriction=mu)
    #p.changeDynamics(robot_id, robot.joint_dict["LAnkleRoll"].index, lateralFriction=mu)

    # ground_id = 0  # usa el ID del plano que encontraste
    # dynamics = p.getDynamicsInfo(ground_id, -1)
    # lateral_friction = dynamics[1]
    # print(f"Lateral friction of ground: {lateral_friction}")

    #time.sleep(5)

    # Crear lista de nombres de joints válidos (sin dedos)
    joint_names = [
        name for name in robot.joint_dict.keys()
        if "Finger" not in name and "Thumb" not in name
    ]


    # joint_velocity_limits = {}

    # for name, joint in robot.joint_dict.items():
    #     if "Finger" not in name and "Thumb" not in name:
    #         try:
                  #jaja así es
    #             max_vel = joint.getMaxVelocity()  # Intenta acceder al límite
    #         except AttributeError:
    #             # Si no tiene ese método, usamos pybullet directamente
    #             joint_index = joint.index
    #             joint_info = p.getJointInfo(robot.uid, joint_index)
    #             max_vel = joint_info[11]

    #         joint_velocity_limits[name] = max_vel
    #         print(f"{name}: {max_vel:.3f} rad/s")



    data = np.load(f"/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/output_trajectory/{dir}/q.npz")
    q = data['q_full']


    qs = q[6:32, 0]  # Tomar los últimos 26 DoF del instante i
    qs1 = q[6:32,1]
    rate = np.ones(26)
    robot.setAngles(joint_names, qs.tolist(), 0.5)
    print("Iniciando movimiento")

    time.sleep(3)
    p.setRealTimeSimulation(1)

    try:
        print(len(q[0,:]))
        for i in range(len(q[0,:])):
            qs = q[6:32, i]  # Tomar los últimos 26 DoF del instante i
            # qs1 = q[6:32,i+1]
            # dqs = (qs1 - qs) / 0.05
            # rate = np.abs(dqs) / dq_max
            rate = np.ones(26)*1
            robot.setAngles(joint_names, qs.tolist(), rate.tolist())


            
            time.sleep(0.05)  # para dar tiempo a la simulación real
        

        final_q = q[-26:, -1].tolist()
        robot.setAngles(joint_names, final_q, rate.tolist())
        time.sleep(5.0)

    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        simulation_manager.stopSimulation(client)

