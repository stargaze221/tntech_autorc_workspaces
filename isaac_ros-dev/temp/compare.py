import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os
from mpl_toolkits.mplot3d import Axes3D

# === Constants ===
WHEELBASE = 0.38  # meters
STEERING_NEUTRAL = 90  # neutral PWM value

# === Rosbag folders (absolute paths) ===
rosbag_folders = [
    "/home/autovhc/workspaces/isaac_ros-dev/rosbag2_2025_06_23-14_06_19_Forward",
    "/home/autovhc/workspaces/isaac_ros-dev/rosbag2_2025_06_23-14_19_35_Rotation",
    "/home/autovhc/workspaces/isaac_ros-dev/rosbag2_2025_06_25-13_11_41",
    "/home/autovhc/workspaces/isaac_ros-dev/rosbag2_2025_06_25-13_12_41",
]

# === 3D plot setup ===
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

# === Process each rosbag folder ===
for folder in rosbag_folders:
    # File paths
    odom_path = os.path.join(folder, "csv_output", "odom_with_time.csv")
    servo_path = os.path.join(folder, "csv_output", "servo_cmd_with_time.csv")

    # Load CSVs
    odom_df = pd.read_csv(odom_path)
    servo_df = pd.read_csv(servo_path)

    # Time alignment
    odom_df["real_time_sec"] = odom_df["time_sec"]
    servo_df["real_time_sec"] = servo_df["time_sec"]

    # Interpolation
    interp_odom = interp1d(odom_df["real_time_sec"], odom_df["linear_x"], fill_value="extrapolate")
    interp_angul = interp1d(odom_df["real_time_sec"], odom_df["angular_z"], fill_value="extrapolate")
    interp_servo = interp1d(servo_df["real_time_sec"], servo_df["data"], fill_value="extrapolate")

    # Common time range
    t_start = max(odom_df["real_time_sec"].min(), servo_df["real_time_sec"].min())
    t_end = min(odom_df["real_time_sec"].max(), servo_df["real_time_sec"].max())
    t_common = np.linspace(t_start, t_end, 500)

    # Interpolated values
    linear_velocity = interp_odom(t_common)
    angul_from_odom = interp_angul(t_common)
    servo_pwm = interp_servo(t_common)

    # Convert PWM to steering angle
    steering_deg = servo_pwm - STEERING_NEUTRAL

    # Label
    label_prefix = folder.replace("/home/autovhc/workspaces/isaac_ros-dev/rosbag2_2025_", "").replace("-", "_")

    # === 3D plot ===
    ax.plot(
        steering_deg,         # X-axis: steering angle
        linear_velocity,      # Y-axis: linear velocity
        angul_from_odom,      # Z-axis: angular velocity from odom
        label=label_prefix
    )

# === Axes labels and formatting ===
ax.set_xlabel("Steering Angle (deg)")
ax.set_ylabel("Linear Velocity (m/s)")
ax.set_zlabel("Angular Velocity from Odom (rad/s)")
ax.set_title("3D Plot: Steering vs Linear vs Angular Velocity (Odom)")
ax.legend()
plt.tight_layout()
plt.show()
