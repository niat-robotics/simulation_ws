# 🧩 simulation_ws

A ROS 2 workspace that simulates the Tortoise Bot robot inside custom Gazebo worlds, including camera support and multiple pre-built world models. It also includes OpenCV-based Python scripts for image viewing and computer vision experiments.

## 📘 Table of Contents

- [Overview](#-overview)
- [Workspace Structure](#-workspace-structure)
- [Packages](#-packages)
- [Prerequisites](#️-prerequisites)
- [Build Instructions](#-build-instructions)
- [Launching the Simulation](#-launching-the-simulation)
- [Scripts and Vision Tools](#-scripts-and-vision-tools)
- [Usage Examples](#-usage-examples)
- [Troubleshooting](#-troubleshooting)
- [License](#-license)
- [Acknowledgements](#-acknowledgements)

## 🧭 Overview

This repository, **simulation_ws**, provides a complete simulation setup in ROS 2 Humble (compatible with Gazebo 11). It is designed for robotics development and experimentation using the Tortoise Bot robot model, which includes:

- A camera for visual sensing
- Multiple environment models (e.g., Nxtwave_world, turtlebot3_house, etc.)
- Launch files for running Gazebo with different environments

Additionally, it contains Python scripts for real-time image processing and computer vision demonstrations.

## 🗂 Workspace Structure

```
simulation_ws/
├── src/
│   ├── tortoise_bot/              # Main ROS 2 package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/               # C++ headers (if used)
│   │   ├── src/                   # C++ source files (if used)
│   │   ├── urdf/                  # URDF/Xacro robot descriptions
│   │   ├── worlds/                # Custom Gazebo world files
│   │   ├── models/                # Gazebo models
│   │   │   ├── Nxtwave_world/
│   │   │   ├── turtlebot3_house/
│   │   │   ├── turtlebot3_burger_cam/
│   │   │   ├── cafe_table/
│   │   │   ├── final_logo_wall/
│   │   │   └── turtlebot3_common/
│   │   └── launch/
│   │       ├── empty_world.launch.py
│   │       ├── simulation.launch.py
│   │       ├── robot_state_publisher.launch.py
│   │       ├── spawn_turtlebot3.launch.py
│   │       └── spawn_test_world.launch.py
│   └── scripts/                   # Python vision utilities
│       ├── cv.py
│       └── image_viewer.py
├── build/
├── install/
└── log/
```

## 🧩 Packages

### 1️⃣ tortoise_bot

This is the main ROS 2 package containing:

- URDFs, meshes, and models for the Tortoise Bot robot
- Launch files to bring up Gazebo simulations
- Custom worlds (Nxtwave_world, turtlebot3_house, etc.)
- **Build type:** `ament_cmake`

### 2️⃣ scripts

A collection of Python tools (not ROS packages) for image processing:

- **cv.py**: demonstrates computer vision (e.g., object detection, filtering, or color tracking)
- **image_viewer.py**: opens and displays live camera feeds or saved images

You can run these scripts independently after sourcing your workspace.

## ⚙️ Prerequisites

Before building, ensure you have:

- **ROS 2 Humble** (or compatible)
- **Gazebo 11**
- **colcon** build tool
- **gazebo_ros**, **xacro**, and other dependencies
- **Python 3** with OpenCV: `pip install opencv-python`

Source ROS:

```bash
source /opt/ros/humble/setup.bash
```

## Clone repository

```bash
git clone https://github.com/niat-robotics/simulation_ws
```

## 🏗 Build Instructions

```bash
cd simulation_ws
colcon build --symlink-install
source install/setup.bash
```

If dependencies are missing:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 🚀 Launching the Simulation

### 1️⃣ Launch with Nxtwave World with Turtlebot3 Cam Model

```bash
export TURTLEBOT3_MODEL = burger_cam
ros2 launch tortoise_bot simulation.launch.py
```

Loads your custom Nxtwave World and spawns the Tortoise Bot robot equipped with a camera.

### 2️⃣ Launch the Robot State Publisher

```bash
ros2 launch tortoise_bot robot_state_publisher.launch.py
```

Publishes TF and joint states of your robot for RViz visualization.

### 3️⃣ Test World Launch

```bash
ros2 launch tortoise_bot spawn_test_world.launch.py
```

Loads a test environment for debugging URDF and plugin behavior.

## 🎥 Scripts and Vision Tools

From the scripts directory:

```bash
cd simulation_ws/src/scripts
python3 cv.py
```

or

```bash
python3 image_viewer.py
```

These can display images, process frames, or visualize live camera topics depending on your implementation.

## 🧠 Usage Examples

### 🕹 Manual Control

Run keyboard teleop:

```bash
export TURTLEBOT3_MODEL = burger
ros2 run turtlebot3_teleop teleop_twist_keyboard
```

Use **W, A, S, D** keys to move the robot.

### 🧭 Autonomous Testing

You can integrate your own ROS 2 nodes (e.g., Safety Node, Follow-the-Gap, Pure Pursuit) with this simulation:

1. Place new packages under `src/`
2. Add them to your workspace build
3. Extend your launch files to start both the simulation and the algorithm nodes together

## 🧾 Troubleshooting

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| Robot not spawning | Wrong path in launch file | Check URDF/Xacro file path and Gazebo plugin names |
| Missing world assets | Model paths incorrect | Verify file paths in `.world` and `.sdf` files |
| Build fails | Missing dependencies | Run `rosdep install` before building |
| Gazebo opens blank | Gazebo paths or environment not sourced | Ensure `GAZEBO_MODEL_PATH` includes `tortoise_bot/models` |

## 🪪 License

**MIT License**

Copyright (c) 2025 Mustaeen

## 🙏 Acknowledgements

- **TurtleBot3 community** for model inspiration
- **ROS 2 and Gazebo teams** for powerful simulation tools

---

**Developed and maintained by Mustaeen** 🧑‍💻
