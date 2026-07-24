# Panda Robot OBB Detection with MoveIt 2

ROS 2 workspace for Franka Emika Panda robot arm with oriented bounding box (OBB) detection using YOLOv8-OBB.

## Packages

- **yolov8_obb** — YOLOv8-OBB inference node for detecting oriented objects from camera feed
- **yolov8_obb_msgs** — Custom ROS 2 messages for YOLOv8 inference results
- **cube_detection** — Color-based cube detection using OpenCV + cv_bridge
- **panda_moveit_config** — MoveIt 2 configuration for the Panda robot (grasping, planning, Gazebo simulation)
- **robot_description** — Panda robot URDF/SDF model files
- **UI** — PyQt5 GUIs for bolt/cube selection and robot control

## Quick Start

```bash
colcon build
source install/setup.bash
ros2 launch panda_moveit_config moveit_gazebo_obb.py
```
