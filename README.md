# SCARA Robot Kinematics and Trajectory Planning

## Overview![Screenshot from 2025-04-06 23-41-13](https://github.com/user-attachments/assets/f715cfaa-9b80-481e-94b5-08bdfec41270)



https://github.com/user-attachments/assets/844a91c0-4a88-4e94-986a-2fa489d055b9


This repository contains the implementation of SCARA robot kinematics and trajectory planning algorithms. The project explores the forward and inverse kinematics of a SCARA robot using MATLAB, providing a comprehensive approach to robotic control. It also includes workspace visualization, trajectory planning in 2D and 3D, and Jacobian calculations using the transpose method for controlling the robot’s end effector.

The project integrates both simulation and real-time control strategies for SCARA robots, making it suitable for researchers and practitioners in robotics and automation.

## Project Structure
The repository consists of several MATLAB scripts and Simulink models that cover the following concepts:

- **Kinematics**: Forward and inverse kinematics algorithms
![Screenshot from 2025-04-06 23-41-42](https://github.com/user-attachments/assets/23e4e1f6-ee84-4e44-97b3-79b5c45571ce)
![Screenshot from 2025-04-06 23-41-55](https://github.com/user-attachments/assets/2a8b4099-e442-4517-a979-a865cb5987d9)

  
- **Workspace**: Visualization of the reachable workspace of the robot
  ![Screenshot from 2025-04-06 23-42-25](https://github.com/user-attachments/assets/acfbf1f1-5b90-4e16-beaf-b000f5a2af1f)
![Screenshot from 2025-04-06 23-42-37](https://github.com/user-attachments/assets/a10db6ef-139d-4598-8624-316eb91b032a)
![Screenshot from 2025-04-06 23-42-49](https://github.com/user-attachments/assets/588acb5d-f336-4cee-a869-92e7e554c151)

- **Trajectory Planning**: Path planning in 2D and 3D, with a focus on SCARA robots
  ![Screenshot from 2025-04-06 23-45-01](https://github.com/user-attachments/assets/9d542731-d231-4af0-bbe3-0f6039278c6e)

- **Jacobia![Screenshot from 2025-04-06 23-45-15](https://github.com/user-attachments/assets/fb4dadde-ac2f-47be-af09-d28840049763)
![Screenshot from 2025-04-06 23-45-26](https://github.com/user-attachments/assets/7418d59a-2ddb-4403-8c82-5c18094d7396)

  
n Calculations**: Use of Jacobian transpose for robot control
- **Simulink Model**: A Simulink model for controlling the robot’s motion using Jacobian transpose

![Screenshot from 2025-04-06 23-45-55](https://github.com/user-attachments/assets/2b25ff1f-dc82-4706-bf81-40f0abc92f08)

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


