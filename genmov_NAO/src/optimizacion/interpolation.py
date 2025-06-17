import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# ----- CONFIG -----
input_file = "warmstart_brazos.npz"           # Input file


FPS = 40        # Hz
DURATION = 5   # seconds
JOINT_NAMES = ["xbase","ybase","zbase","roll","pitch","yaw",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
    "LHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
    "RHand", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    "HeadYaw", "HeadPitch"
]

# ----- LOAD ORIGINAL TRAJECTORY -----
data = np.load(input_file)
q_nao = data["q_full"]  # Shape: 32 x N


num_joints, num_points = q_nao.shape
original_times = np.linspace(0, DURATION, num_points)
resample_times = np.linspace(0, DURATION, int(FPS * DURATION))

# ----- SMOOTH & RESAMPLE -----
q_smooth = np.zeros((num_joints, len(resample_times)))
for j in range(num_joints):
    spline = make_interp_spline(original_times, q_nao[j, :], k=3)
    q_smooth[j, :] = spline(resample_times)

# ----- SAVE OUTPUT -----
print(q_smooth.shape)

np.savez("q_smooth.npz", q_smooth = q_smooth)

# ----- PLOT -----
plt.figure(figsize=(14, 10))
for i in range(num_joints):
    plt.plot(resample_times, q_smooth[i], label=JOINT_NAMES[i])
plt.title("Smoothed Joint Trajectories for NAO")
plt.xlabel("Time [s]")
plt.ylabel("Joint Angle [rad]")
plt.legend(fontsize='small', ncol=2)
plt.grid(True)
plt.tight_layout()
plt.show()
