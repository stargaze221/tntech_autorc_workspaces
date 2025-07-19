import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from sklearn.linear_model import LinearRegression
import os

# === rosbag 폴더 리스트 ===
rosbag_folders = [
    "rosbag2_2025_06_23-14_06_19_forward",
    "rosbag2_2025_06_23-14_19_35_rotation",
    "rosbag2_2025_06_25-13_11_41",
    "rosbag2_2025_06_25-13_12_41",
]

# === 계수 정의 ===
k = 0.001102  # RPM to linear velocity factor

# === 그래프 설정 ===
plt.figure(figsize=(10, 7))

# === 각 rosbag 폴더 반복 ===
for folder in rosbag_folders:
    odom_path = os.path.join(folder, "csv_output", "odom_with_time.csv")
    rpm_path = os.path.join(folder, "csv_output", "vesc_rpm_cmd_with_time.csv")

    # Load CSV
    odom_df = pd.read_csv(odom_path, header=0)
    rpm_df = pd.read_csv(rpm_path, header=0)

    odom_df["real_time_sec"] = odom_df["time_sec"]
    rpm_df["real_time_sec"] = rpm_df["time_sec"]

    # Interpolation
    interp_odom = interp1d(odom_df["real_time_sec"], odom_df["linear_x"].astype(float), kind="linear", fill_value="extrapolate")
    interp_rpm = interp1d(rpm_df["real_time_sec"], rpm_df["data"].astype(float), kind="linear", fill_value="extrapolate")

    # 공통 시간 범위
    t_start = max(odom_df["real_time_sec"].min(), rpm_df["real_time_sec"].min())
    t_end = min(odom_df["real_time_sec"].max(), rpm_df["real_time_sec"].max())
    t_common = np.linspace(t_start, t_end, 500)

    # 보간 결과
    odom_values = interp_odom(t_common)
    rpm_values = interp_rpm(t_common)

    # 정렬
    sorted_idx = np.argsort(rpm_values)
    rpm_sorted = rpm_values[sorted_idx].reshape(-1, 1)
    odom_sorted = odom_values[sorted_idx]

    # 회귀선
    model = LinearRegression().fit(rpm_sorted, odom_sorted)
    odom_pred = model.predict(rpm_sorted)

    # 추정 속도
    estimated_velocity = rpm_sorted.flatten() * k

    # 라벨 접두사
    label_prefix = folder.replace("rosbag2_2025_", "").replace("-", "_")

    # 그래프 그리기
    plt.plot(rpm_sorted, odom_sorted, label=f"{label_prefix} (data)", linewidth=1.8)
    plt.plot(rpm_sorted, odom_pred, linestyle="--", label=f"{label_prefix} (fit, k={model.coef_[0]:.4f})")
    plt.plot(rpm_sorted, estimated_velocity, linestyle=":", label=f"{label_prefix} (est, k=0.001102)")

# === plot 마무리 ===
plt.xlabel("RPM Command")
plt.ylabel("Linear Velocity (m/s)")
plt.title("Multiple Rosbag: RPM vs Linear Velocity with Regression & Estimated")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
