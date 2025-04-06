# SCARA Robot Kinematics and Trajectory Planning

## Overview
This repository contains the implementation of SCARA robot kinematics and trajectory planning algorithms. The project explores the forward and inverse kinematics of a SCARA robot using MATLAB, providing a comprehensive approach to robotic control. It also includes workspace visualization, trajectory planning in 2D and 3D, and Jacobian calculations using the transpose method for controlling the robot’s end effector.

The project integrates both simulation and real-time control strategies for SCARA robots, making it suitable for researchers and practitioners in robotics and automation.

## Project Structure
The repository consists of several MATLAB scripts and Simulink models that cover the following concepts:

- **Kinematics**: Forward and inverse kinematics algorithms
- **Workspace**: Visualization of the reachable workspace of the robot
- **Trajectory Planning**: Path planning in 2D and 3D, with a focus on SCARA robots
- **Jacobian Calculations**: Use of Jacobian transpose for robot control
- **Simulink Model**: A Simulink model for controlling the robot’s motion using Jacobian transpose

## Features
- **Forward and Inverse Kinematics**: Algorithms for calculating the position and orientation of the end effector.
- **Workspace Analysis**: Computes and visualizes the reachable workspace of the SCARA robot in 3D.
- **Trajectory Planning**: Includes path planning in XY, ZX, and 3D space, and visualizes the robot’s movement.
- **Jacobian Transpose Method**: Implements inverse kinematics using the Jacobian transpose for real-time control.
- **Simulink Model**: A Simulink model for visualizing robot motion control.

## Src
- **Direct_Kinematics.m**: Script for calculating forward kinematics.
- **Inverse_Kinematics.m**: Script for calculating inverse kinematics.
- **Reachable_WorkSpace.m**: Script for visualizing the reachable workspace.
- **threeD_Trajectory_planning.m**: Script for 3D trajectory planning.
- **toolbox_elaborate.m**: Helper functions for kinematic calculations.
- **Trajectory_planning.m**: Script for defining trajectory path and motion.
- **Velocities_and_accelerations.m**: Computes velocities and accelerations for trajectory planning.
- **Simulink model**: `Jacobian_Transpose.slx` for real-time control of the robot.


