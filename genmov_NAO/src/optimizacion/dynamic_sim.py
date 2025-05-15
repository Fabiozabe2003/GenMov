import pybullet as p
from qibullet import SimulationManager
import time
import numpy as np
from nao_limits import *

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

    # Crear lista de nombres de joints válidos (sin dedos)
    joint_names = [
        name for name in robot.joint_dict.keys()
        if "Finger" not in name and "Thumb" not in name
    ]



    # Vector de ángulos a aplicar (debe tener misma longitud que joint_names)
    # Puedes reemplazar estos valores con los que necesites

    
    data = np.load("/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/warmstart_brazos.npz")

    q = data['q_full']
    qs =  np.array(q)

    time.sleep(5)
    p.setRealTimeSimulation(1)

    try:
        print(len(q[0,:]))
        for i in range(len(q[0,:])-1):
            qs = q[6:32, i]  # Tomar los últimos 26 DoF del instante i
            qs1 = q[6:32,i+1]
            dqs = (qs1 - qs) / 0.05
            rate = np.abs(dqs) / dq_max
            rate = np.ones(26)*1
            robot.setAngles(joint_names, qs.tolist(), rate.tolist())
            time.sleep(0.075)  # para dar tiempo a la simulación real
            #robot.angleInterpolation(joint_names,qs.tolist(), rate.tolist())

        final_q = q[-26:, -1].tolist()
        while True:
            robot.setAngles(joint_names, final_q, rate.tolist())
            time.sleep(0.075)
            #robot.angleInterpolation(joint_names, final_q, rate.tolist())

    except KeyboardInterrupt:
        pass
    finally:
        simulation_manager.stopSimulation(client)
