# tntech_autorc_workspaces


## Git Clone and Update Submodule
Since the custom envrionment that we worked on has substantial content, we added a forked isaac_ros_common as submodule. So, after git-clone the entire workspace, you need the following

```bash
git submodule update --init --recursive
```

## Update Bashrc
For convenience, add the following to .bashrc
```bash
export ISAAC_ROS_WS=/home/yoonlab02agxcar/tntech_autorc_workspaces/isaac_ros-dev/

alias isaacdev='cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh -d ${ISAAC_ROS_WS}'
```

## Install NVBlox and Realsense example
For 3D mapping, we rely on NV Blox. See the link below to install the example with Realsense Camera.
https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_realsense.html

## Udev Rule for VESC, Lidar, and Arduino



# Other information
https://jetsonhacks.com/2025/07/12/why-chromium-suddenly-broke-on-jetson-orin-and-how-to-bring-it-back/