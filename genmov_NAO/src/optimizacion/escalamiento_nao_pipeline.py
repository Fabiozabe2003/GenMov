import bvhio
import pandas as pd
import numpy as np
import sys
import os
import time
t_init=time.time()

#dir=sys.argv[1]
dir="caso3"
print(dir)
directory = '/home/invitado8/momask-codes/motions/nao/{}'.format(dir)
files = os.listdir(directory)
file=files[0] #En este caso es el que no es ik
path=os.path.join(directory, file)
root = bvhio.readAsHierarchy(path)

nframes = len(root.Keyframes)

#Relaciones con bvh
hips_ = root.filter('Hips')[0]

lshoulder_ = root.filter('LeftShoulder')[0]
larm_ = root.filter('LeftArm')[0]
lforearm_ = root.filter('LeftForeArm')[0]
lhand_ = root.filter('LeftHand')[0]

rshoulder_ = root.filter('RightShoulder')[0]
rarm_ = root.filter('RightArm')[0]
rforearm_ = root.filter('RightForeArm')[0]
rhand_ = root.filter('RightHand')[0]

lhip_ = root.filter('LeftUpLeg')[0]
lknee_=root.filter('LeftLeg')[0]
lankle_=root.filter('LeftFoot')[0]
lfoot_=root.filter('LeftToe')[0]

rhip_ = root.filter('RightUpLeg')[0]
rknee_=root.filter('RightLeg')[0]
rankle_=root.filter('RightFoot')[0]
rfoot_=root.filter('RightToe')[0]

# Vectores vacíos
larm = []; rarm = []
lshoulder = []; rshoulder = []
lforearm = []; rforearm = []
lhand = []; rhand = []

lhip = []; rhip = []
lknee = []; rknee = []
lankle = []; rankle = []
lfoot = []; rfoot = []

# Leer valores
for i in range(nframes):
    hips_.loadPose(i)

    lshoulder_.loadPose(i)
    larm_.loadPose(i)
    lforearm_.loadPose(i)
    lhand_.loadPose(i)

    rshoulder_.loadPose(i)
    rarm_.loadPose(i)
    rforearm_.loadPose(i)
    rhand_.loadPose(i)

    lhip_.loadPose(i)
    lknee_.loadPose(i)
    lankle_.loadPose(i)
    lfoot_.loadPose(i)

    rhip_.loadPose(i)
    rknee_.loadPose(i)
    rankle_.loadPose(i)
    rfoot_.loadPose(i)

    lshoulder.append([lshoulder_.PositionWorld.x, lshoulder_.PositionWorld.y, lshoulder_.PositionWorld.z])
    larm.append([larm_.PositionWorld.x, larm_.PositionWorld.y, larm_.PositionWorld.z])
    lforearm.append([lforearm_.PositionWorld.x, lforearm_.PositionWorld.y, lforearm_.PositionWorld.z])
    lhand.append([lhand_.PositionWorld.x, lhand_.PositionWorld.y, lhand_.PositionWorld.z])

    rshoulder.append([rshoulder_.PositionWorld.x, rshoulder_.PositionWorld.y, rshoulder_.PositionWorld.z])
    rarm.append([rarm_.PositionWorld.x, rarm_.PositionWorld.y, rarm_.PositionWorld.z])
    rforearm.append([rforearm_.PositionWorld.x, rforearm_.PositionWorld.y, rforearm_.PositionWorld.z])
    rhand.append([rhand_.PositionWorld.x, rhand_.PositionWorld.y, rhand_.PositionWorld.z])

    lhip.append([lhip_.PositionWorld.x, lhip_.PositionWorld.y, lhip_.PositionWorld.z])
    lknee.append([lknee_.PositionWorld.x, lknee_.PositionWorld.y, lknee_.PositionWorld.z])
    lankle.append([lankle_.PositionWorld.x, lankle_.PositionWorld.y, lankle_.PositionWorld.z])
    lfoot.append([lfoot_.PositionWorld.x, lfoot_.PositionWorld.y, lfoot_.PositionWorld.z])

    rhip.append([rhip_.PositionWorld.x, rhip_.PositionWorld.y, rhip_.PositionWorld.z])
    rknee.append([rknee_.PositionWorld.x, rknee_.PositionWorld.y, rknee_.PositionWorld.z])
    rankle.append([rankle_.PositionWorld.x, rankle_.PositionWorld.y, rankle_.PositionWorld.z])
    rfoot.append([rfoot_.PositionWorld.x, rfoot_.PositionWorld.y, rfoot_.PositionWorld.z])

lshoulder = np.array(lshoulder)
larm = np.array(larm)
lforearm = np.array(lforearm)
lhand = np.array(lhand)

rshoulder = np.array(rshoulder)
rarm = np.array(rarm)
rforearm = np.array(rforearm)
rhand = np.array(rhand)

lhip = np.array(lhip)
lknee = np.array(lknee)
lankle = np.array(lankle)
lfoot = np.array(lfoot)

rhip = np.array(rhip)
rknee = np.array(rknee)
rankle = np.array(rankle)
rfoot = np.array(rfoot)

# Contacts
contact_l=[1]*(nframes)
contact_r=[1]*(nframes)

for i in range(5,nframes):
    xl,yl,zl=lfoot[i]
    xr,yr,zr=rfoot[i]
    th=0.07 # 0.03
    if (yl>th and yr>th):
        print(yl,yr)
        if yl>yr:
            contact_l[i]=0
        else: 
            contact_r[i]=0          
    else:
        if yl>th:
            contact_l[i]=0
        elif yr>th:
            contact_r[i]=0

# Para las articulaciones del Robot:
lshoulderRobot = np.zeros((nframes,3));rshoulderRobot = np.zeros((nframes,3))
larmRobot = np.zeros((nframes,3));rarmRobot = np.zeros((nframes,3))
lforearmRobot = np.zeros((nframes,3));rforearmRobot = np.zeros((nframes,3))
lhandRobot = np.zeros((nframes,3));rhandRobot = np.zeros((nframes,3))

lhipRobot = np.zeros((nframes,3));rhipRobot = np.zeros((nframes,3))
lkneeRobot = np.zeros((nframes,3));rkneeRobot = np.zeros((nframes,3))
lankleRobot = np.zeros((nframes,3));rankleRobot = np.zeros((nframes,3))
lfootRobot = np.zeros((nframes,3));rfootRobot = np.zeros((nframes,3))

for frame_number in range(nframes):
        #Fabio dimensions
        k=1
        k1 = k*(9.8)/9.635; k2= k*(10.5+1.5)/26.137; k3=k*(5.775+5.595)/24.94
        k4 = k*(10.0)/37.68; k5= k*(10.29)/40.058; k6=k*(4.519)/13.43
                
        # Puntos en humano
        lSh = lshoulder[frame_number]; rSh = rshoulder[frame_number]
        lAh = larm[frame_number]; rAh = rarm[frame_number]
        lFh = lforearm[frame_number]; rFh = rforearm[frame_number]
        lHh = lhand[frame_number]; rHh = rhand[frame_number]
        lHih = lhip[frame_number]; rHih = rhip[frame_number]
        lKh = lknee[frame_number]; rKh = rknee[frame_number]
        lAnh = lankle[frame_number]; rAnh = rankle[frame_number]
        lFoh = lfoot[frame_number]; rFoh = rfoot[frame_number]


        # Escalamiento para puntos del robot
        #lSr = [0.098,0.1,0]#lSh#[0, 0.1, 0] 
        lSr = [0,0.1,0]
        lAr = (lAh-lSh)*k1 + lSr#r
        lFr = (lFh - lAh)*k2 + lAr
        lHr = (lHh - lFh)*k3 + lFr
        rSr = [0,0.1,0]#[0, 0.1, 0] 
        rAr = (rAh-rSh)*k1 + rSr#r
        rFr = (rFh - rAh)*k2 + rAr
        rHr = (rHh - rFh)*k3 + rFr

        lHir = [0.050,-0.085,0] 
        lKr = (lKh-lHih)*k4 + lHir#r
        lAnr = (lAnh - lKh)*k5 + lKr
        lFor = (lFoh - lAnh)*k6 + lAnr
        rHir = [-0.050,-0.085,0] 
        rKr = (rKh-rHih)*k4 + rHir#r
        rAnr = (rAnh - rKh)*k5 + rKr
        rFor = (rFoh - rAnh)*k6 + rAnr

        # Puntos en robot
        lshoulderRobot[frame_number,:] = lSr
        larmRobot[frame_number,:] = lAr 
        lforearmRobot[frame_number,:] = lFr
        lhandRobot[frame_number,:] = lHr
        lhipRobot[frame_number,:] = lHir
        lkneeRobot[frame_number,:] = lKr 
        lankleRobot[frame_number,:] = lAnr
        lfootRobot[frame_number,:] = lFor

        rshoulderRobot[frame_number,:] = rSr
        rarmRobot[frame_number,:] = rAr 
        rforearmRobot[frame_number,:] = rFr
        rhandRobot[frame_number,:] = rHr
        rhipRobot[frame_number,:] = rHir
        rkneeRobot[frame_number,:] = rKr 
        rankleRobot[frame_number,:] = rAnr
        rfootRobot[frame_number,:] = rFor
# Reorder columns
orden = [2, 0, 1 ] #Cambio de coordenadas (Rviz)
lshoulderRobot = lshoulderRobot[:,orden]
larmRobot = larmRobot[:,orden]
lforearmRobot = lforearmRobot[:,orden]
lhandRobot = lhandRobot[:,orden]
lhipRobot = lhipRobot[:,orden]
lkneeRobot = lkneeRobot[:,orden]
lankleRobot = lankleRobot[:,orden]
lfootRobot = lfootRobot[:,orden]
rshoulderRobot = rshoulderRobot[:,orden]
rarmRobot = rarmRobot[:,orden]
rforearmRobot = rforearmRobot[:,orden]
rhandRobot= rhandRobot[:,orden]
rhipRobot = rhipRobot[:,orden]
rkneeRobot = rkneeRobot[:,orden]
rankleRobot = rankleRobot[:,orden]
rfootRobot = rfootRobot[:,orden]


num_frames=nframes

body_parts = ["lhand", "lforearm", "lshoulder", "larm", "lhip", "lknee", "lankle", "lfoot", "rshoulder", "rarm", "rforearm", "rhand", "rhip", "rknee", "rankle", "rfoot"]

# Create a dictionary to map body parts to their data
body_part_data = {
    "lhand": lhandRobot,
    "lforearm": lforearmRobot,
    "lshoulder": lshoulderRobot,
    "larm": larmRobot,
    "lhip": lhipRobot,
    "lknee": lkneeRobot,
    "lankle": lankleRobot,
    "lfoot": lfootRobot,
    "rshoulder": rshoulderRobot,
    "rarm": rarmRobot,
    "rforearm": rforearmRobot,
    "rhand": rhandRobot,
    "rhip": rhipRobot,
    "rknee": rkneeRobot,
    "rankle": rankleRobot,
    "rfoot": rfootRobot,
}
data = []


for body_part, points in body_part_data.items():
    for frame_idx in range(num_frames//3,num_frames):
        # Get the x, y, z coordinates for the current body part and frame
        x, y, z = points[frame_idx]
        # Append the data for the current frame and body part
        data.append({
            "frame": frame_idx,
            "body_part": body_part,
            "x": x,
            "y": y,
            "z": z,
            "left_contact": contact_l[frame_idx],
            "right_contact": contact_r[frame_idx]
        })

# Create a DataFrame from the collected data
df = pd.DataFrame(data)

# Save the DataFrame to a CSV file
filename = '/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/motions/{dir}.csv'.format(dir=dir)
df.to_csv(filename, index=False)
print(f"Motion data saved to {filename}")

# Save time
t_fin=time.time()
filename=f'/home/invitado8/proy_ws/src/GenMov/genmov_NAO/src/optimizacion/times/{dir}.txt'
with open(filename,'r') as file:
    lines = file.readlines()
while len(lines) < 2:
    lines.append("\n")
lines[1]=f"{t_fin-t_init}\n"

with open(filename, 'w') as file:
    file.writelines(lines)
print("Written to line 1")
