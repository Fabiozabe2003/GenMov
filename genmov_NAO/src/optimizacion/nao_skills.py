import qi
import cv2
import numpy as np
#from ultralytics import YOLO
from forward_kinematics import *


def foot_bumper_pressed(session):
    memory = session.service("ALMemory")

    # Ensure that ALMemory is running
    # Get the values of the foot bumpers
    left_bumper = memory.getData("Device/SubDeviceList/LFoot/Bumper/Left/Sensor/Value")
    right_bumper = memory.getData("Device/SubDeviceList/RFoot/Bumper/Left/Sensor/Value")

    # Check if any bumper is pressed
    return left_bumper == 1.0 or right_bumper == 1.0


def get_distance(session):
    
    sonar = session.service("ALSonar")  
    memory = session.service("ALMemory")  

    # Subscribe to sonar sensor to start measuring
    sonar.subscribe("SonarApp") #"MyApplication?"

    # Maybe a delay?
    # Read sonar values from ALMemory
    left_distance = memory.getData("Device/SubDeviceList/US/Left/Sensor/Value")
    right_distance = memory.getData("Device/SubDeviceList/US/Right/Sensor/Value")

    # Unsubscribe to stop sonar to save energy
    sonar.unsubscribe("SonarApp")

    distances = {
            "left": left_distance,
            "right": right_distance
        }

    return distances


def joints_rarm(session, q):
    """
   q: List of 6 joint angles for the right arm [ShoulderPitch, ShoulderRoll, ElbowYaw, ElbowRoll, WristYaw, Hand]
    """
    
    motion = session.service("ALMotion")
        
    # Define joint names for the right arm
    joint_names = [
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "RHand"
        ]
        
    if len(q) != len(joint_names):
        print("Error: q must have 6 values corresponding to the right arm joints.")
        return

    # Define movement speed (adjust if necessary)
    fraction_max_speed = 0.2  

    # Set the joint angles
    motion.setAngles(joint_names, q, fraction_max_speed)


def joints_larm(session, q):
    """
   q: List of 6 joint angles for the  left  arm [ShoulderPitch, ShoulderRoll, ElbowYaw, ElbowRoll, WristYaw, Hand]
    """
    
    motion = session.service("ALMotion")
        
    # Define joint names for the right arm
    joint_names = [
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbowYaw",
            "LElbowRoll",
            "LWristYaw",
            "LHand"
        ]
        
    if len(q) != len(joint_names):
        print("Error: q must have 6 values corresponding to the left arm joints.")
        return

    # Define movement speed (adjust if necessary)
    fraction_max_speed = 0.2  

    # Set the joint angles
    motion.setAngles(joint_names, q, fraction_max_speed)

def joints_head(session, q):
    """
   q: List of 2 joint angles for the head [HeadYaw, HeadPitch]
    """
    motion = session.service("ALMotion")
        
    # Define joint names for the right arm
    joint_names = [
            "HeadYaw",
            "HeadPitch" ]
        
    if len(q) != len(joint_names):
        print("Error: q must have 2 values corresponding to the head joints.")
        return

    # Define movement speed (adjust if necessary)
    fraction_max_speed = 0.2  

    # Set the joint angles
    motion.setAngles(joint_names, q, fraction_max_speed)


def enable_motors(session):
    motion = session.service("ALMotion")

    # Enable motor stiffness
    motion.stiffnessInterpolation("Head", 1.0, 1.0)
    motion.stiffnessInterpolation("LArm", 1.0, 1.0)
    motion.stiffnessInterpolation("RArm", 1.0, 1.0)
    motion.stiffnessInterpolation("LLeg", 1.0, 1.0)
    motion.stiffnessInterpolation("RLeg", 1.0, 1.0)


def sit_down(session):
    posture = session.service("ALRobotPosture")
    posture.goToPosture("Sit", 0.8)
        #print("Sitting down.")


def walk_forward(session, distance=0.5):   # belocidad ija?
    motion = session.service("ALMotion")

    # Ensure robot is in a standing position
    posture = session.service("ALRobotPosture")
    posture.goToPosture("StandInit", 0.5)

    # Move forward
    motion.moveTo(distance, 0, 0)

    print(f"Walking forward {distance} meters ")


def turn(session, angle):  # belocidad ija
    
    motion = session.service("ALMotion")
    # Ensure robot is in a standing position
    posture = session.service("ALRobotPosture")
    posture.goToPosture("StandInit", 0.5)

    # Turn in place
    motion.moveTo(0, 0, angle)

    print(f"Turning {angle} radians")



def connect_camera(session, camera_id=0, resolution=1, color_space=13, fps=10):
    """
    Connects to NAO's camera and streams the video in real time.

    :param session: Qi session connected to the robot.
    :param camera_id: 0 = Top Camera, 1 = Bottom Camera
    :param resolution: 0=160x120, 1=320x240, 2=640x480, 3=1280x960
    :param color_space: 13 = BGR (for OpenCV)
    :param fps: Frames per second
    """
    try:
        video_service = session.service("ALVideoDevice")

        # Subscribe to the camera
        subscriber_id = video_service.subscribeCamera(
            "camera_test", camera_id, resolution, color_space, fps
        )
        
        print("Connected to camera. Streaming live video...")


        # Obtener matriz intrínseca (fx, 0, cx, 0, fy, cy, 0, 0, 1)
        params = video_service.getCameraParameter(subscriber_id, 18)  # kCameraIntrinsicMatrixID = 18
        print("Intrinsics:", params)


        while True:
            # Get image from NAO camera
            image_container = video_service.getImageRemote(subscriber_id)

            if image_container is None:
                print("Failed to capture image")
                continue

            width = image_container[0]
            height = image_container[1]
            array = np.frombuffer(image_container[6], dtype=np.uint8)  # Convert byte array to numpy
            image = array.reshape((height, width, 3))  # Reshape into (H, W, 3) BGR format
            
            # Display image using OpenCV
            cv2.imshow("NAO Camera", image)

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # Cleanup
        video_service.unsubscribe(subscriber_id)
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"Error in connect_camera: {e}")


def detect_camera(session, camera_id=0, resolution=1, color_space=13, fps=60):
    """
    Connects to NAO's camera, streams video, and runs object detection using YOLO.
    """
    try:
        # Cargar modelo YOLO (best.pt)
        model = YOLO("best.pt")

        video_service = session.service("ALVideoDevice")

        # Subscribe to the camera
        subscriber_id = video_service.subscribeCamera(
            "camera_test", camera_id, resolution, color_space, fps
        )
        
        print("Connected to camera. Streaming live video...")

        while True:
            image_container = video_service.getImageRemote(subscriber_id)

            if image_container is None:
                print("Failed to capture image")
                continue

            width = image_container[0]
            height = image_container[1]
            array = np.frombuffer(image_container[6], dtype=np.uint8)
            image = array.reshape((height, width, 3))

            # Detección con YOLO
            results = model(image, imgsz=640, verbose=False)

            # Ploteamos la detecciones según precisión
            detections = results[0].boxes 
            if len(detections) > 0:
                # Dibujar las cajas delimitadoras y etiquetas en el frame
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        conf = box.conf.item()
                        if conf>0.7:
                            # Dibujar los cuadros en la imagen
                            image = results[0].plot()
            # Mostrar en ventana
            cv2.imshow("NAO Camera + YOLO Detection", image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        video_service.unsubscribe(subscriber_id)
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"Error in connect_camera: {e}")



def capture_images(session, camera_id=0, resolution=1, color_space=13, fps=10, save_key='c'):
    """
    Captures images from NAO's camera and saves them when a key is pressed.

    :param session: Qi session connected to the robot.
    :param camera_id: 0 = Top Camera, 1 = Bottom Camera
    :param resolution: 0=160x120, 1=320x240, 2=640x480, 3=1280x960
    :param color_space: 13 = BGR (for OpenCV)
    :param fps: Frames per second
    :param save_key: Key to press to save the image (default is spacebar)
    """
    try:
        video_service = session.service("ALVideoDevice")
        subscriber_id = video_service.subscribeCamera(
            "photo_capture", camera_id, resolution, color_space, fps
        )

        print("Camera connected. Press SPACE to take a photo, or Q to quit.")
        photo_counter = 0

        while True:
            image_container = video_service.getImageRemote(subscriber_id)

            if image_container is None:
                print("Failed to capture image")
                continue

            width = image_container[0]
            height = image_container[1]
            array = np.frombuffer(image_container[6], dtype=np.uint8)
            image = array.reshape((height, width, 3))

            cv2.imshow("NAO Camera - Press SPACE to save", image)
           
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord(save_key):
                filename = f"nao2_photo_{photo_counter}.jpg"
                cv2.imwrite(filename, image)
                print(f"Saved {filename}")
                photo_counter += 1

        video_service.unsubscribe(subscriber_id)
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"Error in capture_images: {e}")



def object_detection(session, camera_id=0, resolution=1, color_space=13, fps=60):
    """
    Connects to NAO's camera, streams video, and runs object detection using YOLO.
    Returns the center of the first detected object and shows the image for 1 second.
    """
    try:
        # Load YOLO model
        model = YOLO("best.pt")

        video_service = session.service("ALVideoDevice")

        # Subscribe to the camera
        subscriber_id = video_service.subscribeCamera(
            "camera_test", camera_id, resolution, color_space, fps
        )

        print("Connected to camera. Streaming live video...")

        while True:
            image_container = video_service.getImageRemote(subscriber_id)

            if image_container is None:
                print("Failed to capture image")
                continue

            width = image_container[0]
            height = image_container[1]
            array = np.frombuffer(image_container[6], dtype=np.uint8)
            image = array.reshape((height, width, 3))

            # YOLO detection
            results = model(image, imgsz=(320,240), verbose=False)#640, ahora 320
            rsz=1

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    conf = box.conf.item()
                    if conf > 0.5:
                        x1, y1, x2, y2 = box.xyxy[0]
                        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                        # Corners: top-left, top-right, bottom-left, bottom-right
                        corners = np.array([
                            [x1, x2, x1, x2],  # u
                            [y1, y1, y2, y2]   # v
                        ], dtype=np.float32)

                        # Draw rectangle and corners
                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        #for i in range(4):
                        #    cv2.circle(image, (corners[0, i], corners[1, i]), 3, (255, 0, 0), -1)
                        
                        cv2.imshow("Detection", image)
                        cv2.waitKey(1000)
                        cv2.destroyAllWindows()

                        video_service.unsubscribe(subscriber_id)
                        print("Object detected. Returning corners of bounding box.")
                        return corners/rsz
                    

    except Exception as e:
        print(f"Error in object_detection: {e}")
        return None
    

def stereo_vision(session):
    # Instrinsic parameters matrix (k)
    k=np.array([[276.434, 0.0, 163.397],
                [0.0, 275.621, 119.537],
                [0.0, 0.0, 1.0]])
    
    coefs_dist=np.array([2.800e-2,-7.560e-1, -1.000e-2, 2.000e-3, 2.022])
    
    #k=np.array([[241.42, 0.0, 160],
    #            [0.0, 241.42, 120],
     #           [0.0, 0.0, 1.0]])
    #np.pi/9
    q1=[np.pi/10,0.349066]
    joints_head(session,q1)
    #time.sleep(1)
    P1=k@fkine_top_cam(q1,'t')[0:3,:]
    x1 = object_detection(session)
    #x1 = np.array([[[x1[0], x1[1]]]], dtype=np.float32)
    x1 = cv2.undistortPoints(x1, k, coefs_dist)
    #x1 = x1.reshape(2, 1)

    q2=[-np.pi/10,0.349066]
    joints_head(session,q2)
    P2=k@fkine_top_cam(q2,'t')[0:3,:]
    #time.sleep(1)
    #print(P2)
    x2 = object_detection(session)
    #x2 = np.array([[[x2[0], x2[1]]]], dtype=np.float32)
    x2 = cv2.undistortPoints(x2, k, coefs_dist)
    #x2 = x2.reshape(2, 1)

    points_4d = cv2.triangulatePoints(P1, P2, x1, x2)


    points_3d = points_4d[:3] / points_4d[3]
    R = np.array([[1,  0,  0],
              [0, -1,  0],
              [0,  0, -1]])
    R =np.eye(3)


    point_robot_frame = R @ points_3d
    print("3D Point:\n",point_robot_frame)


        
def main():
    robot_ip = "192.168.10.100"  # Replace with your robot's IP
    robot_port = 9559

    # Create an application instance
    app = qi.Application(["NAOqiApp", f"--qi-url=tcp://{robot_ip}:{robot_port}"])
    app.start()  # Start the application
    session = app.session  # Get the session from the application

    try:
        print("Connected to the robot!")

        motion = session.service("ALMotion")

         # Enable motor stiffness
        motion.stiffnessInterpolation("Head", 1.0, 1.0)

        stereo_vision(session)
        #capture_images(session)
        # Call the walk_forward function
        #enable_motors(session)
        #joints_rarm(session, [0.3, -0.2, 1.0, 0.5, 0.3, 0.8])
        #joints_larm(session, [0.3, 0.2, -1.0, -0.5, -0.3, 0.8])
        #turn(session,3.14)
        #sit_down(session)

    except Exception as e:
        print(f"Failed to connect to robot: {e}")
    
    finally:
        # Close the Qi session properly
        print("Closing Qi session...")
        app.stop()

if __name__ == "__main__":
    main()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       