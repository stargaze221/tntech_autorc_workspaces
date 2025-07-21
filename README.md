# `tntech_autorc_workspaces`

## 🧬 Cloning the Repository and Initializing Submodules

This workspace uses a custom environment with substantial modifications, including a forked version of `isaac_ros_common` as a Git submodule. After cloning the repository, be sure to initialize and update all submodules:

```bash
git submodule update --init --recursive
```

---

## ⚙️ Setting Up Your `.bashrc`

For convenience, add the following lines to your `~/.bashrc`:

```bash
export ISAAC_ROS_WS=/home/yoonlab02agxcar/tntech_autorc_workspaces/isaac_ros-dev/

alias isaacdev='cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh -d ${ISAAC_ROS_WS}'
```

Then, apply the changes:

```bash
source ~/.bashrc
```

---

## 🧠 Follow the Official Isaac ROS Setup

Refer to the official NVIDIA Isaac ROS compute setup instructions:

🔗 https://nvidia-isaac-ros.github.io/getting_started/index.html

---

## 🧱 Installing NVBlox with RealSense for 3D Mapping

For 3D scene reconstruction, we use NVBlox in combination with a RealSense camera. Follow the NVBlox + RealSense tutorial here:

🔗 https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_realsense.html

---

## 🔌 Udev Rules for VESC, Lidar, and Arduino

> [Add detailed steps or a file reference here if available]

Ensure persistent access to your VESC, Lidar, and Arduino devices by configuring appropriate udev rules.

---

## 🧾 Additional Notes

- **Chromium issue on Jetson Orin:**  
  A helpful blog post about recent Chromium breakage and how to fix it:  
  🔗 https://jetsonhacks.com/2025/07/12/why-chromium-suddenly-broke-on-jetson-orin-and-how-to-bring-it-back/

- **Isaac ROS config file:**  
  Create or update the following config file:  
  `isaac_ros_common/scripts/.isaac_ros_common-config`  
  with the content:
  ```bash
  CONFIG_IMAGE_KEY=ros2_humble.realsense.onetenthrc
  ```