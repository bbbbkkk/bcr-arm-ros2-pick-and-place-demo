# BCR Arm ROS2 Gazebo MoveIt2 Demo

This project implements a ROS2-based simulation and motion planning demo for the BCR Arm robot.

## Features

- BCR Arm URDF/xacro robot description
- Gazebo simulation environment
- ros2_control integration with joint trajectory controller
- MoveIt2 motion planning configuration
- RViz Plan & Execute support
- MoveIt PlanningScene obstacle objects
- One-command bringup launch
- Automatic MoveIt demo for end-effector obstacle crossing

## Packages

```text
bcr_arm_description
  Robot URDF/xacro, meshes, ros2_control description

bcr_arm_gazebo
  Gazebo world, robot spawn, controller configuration

bcr_arm_moveit_config
  MoveIt2 configuration, SRDF, kinematics, RViz config, PlanningScene script

bcr_arm_bringup
  One-command launch for Gazebo + MoveIt + RViz + obstacles

bcr_arm_demo
  Automatic MoveIt planning and execution demo
