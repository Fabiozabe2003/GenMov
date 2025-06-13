import qi
import sys

def main():
    robot_ip = "169.254.129.144"  # IP del NAO

    # Iniciar sesión con el robot
    try:
        session = qi.Session()
        session.connect("tcp://" + robot_ip + ":9559")
        print("[INFO] Conectado al NAO en " + robot_ip)
    except RuntimeError:
        print("[ERROR] No se pudo conectar al NAO en IP:", robot_ip)
        sys.exit(1)

    # Obtener servicios
    motion = session.service("ALMotion")
    posture = session.service("ALRobotPosture")

    # Activar motores
    motion.wakeUp()

    # Pasar a postura de pie (StandInit)
    posture.goToPosture("StandInit", 0.5)

    print("[INFO] El NAO ahora está de pie.")

if __name__ == "__main__":
    main()
