# UGV Perception and Control Using LIDAR and Model Predictive Control

This repository contains MATLAB and Simulink files developed for implementing perception and control algorithms for unmanned ground vehicles (UGVs), focusing on differential-drive robots. The system uses LIDAR sensor data for mapping, obstacle detection, and navigation control based on Model Predictive Control.

## Overview

The primary goals of this project are:
- Develop a LIDAR-based perception pipeline for obstacle detection and representation.
- Implement a control strategy for UGVs to follow predefined paths using Model Predictive Control (MPC) and other optimal control methods.
- Simulate and validate the complete system in MATLAB/Simulink and Gazebo.

## Repository Structure

| Directory/File       | Description                                               |
|----------------------|-----------------------------------------------------------|
| `Draw_MPC_point_stabilization.m` | MATLAB script to plot 2D the position of the robot without using the ROS/Gazbeo interface. |
| `casadi_block.m`   | MATLAB script for the MATLAB Function block in Simulink. |
| `casadi_script.m`   | MATLAB script used as base for the `casadi_block.m` file. |
| `gera_casadi_function.m`   | MATLAB script used to test the control in Gazebo without Simulink. |
| `shift.m`   | MATLAB script used in the `gera_casadi_function.m` file. |
| `ros_lidar2d` | MATLAB script used to analyse, filter the data from a 2D LIDAR and plot the obstacles as polytopes. |
| `ros_lidar3d` | MATLAB script used to analyse, filter the data from a 3D LIDAR and plot the obstacles as polytopes. |
| `Controltest_gazebo_trajectory_23a.slx`  | Simulink models for communicating with Gazebo through ROS. |
| `MPC_p2p.slx` | Simulink base file for the communication with Gazebo. |


## Dependencies

- MATLAB R2023a or later  
- Simulink
- Casadi Library - version 3.6.5 or later
- MATLAB Toolboxes:
- MPT3
- Ellipsoidal Toolbox (ET)
- ROS (optional, for real hardware communication or Gazebo simulation)
