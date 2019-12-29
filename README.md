# Design of an Active Inference and Kalman filter applied to the Jackal robot simulation in Gazebo 7 using ROS Kinetic
This ROS package contains the following folders:
- launch: includes the launch file needed to run all ROS nodes at once, including the simulation control, Gazebo simulation with correct arguments, data processing, Active Inference algorithm and Kalman filter
- scripts: includes all Python scripts needed to run the simulation control, data processing, Active Inference algorithm and Kalman filter ROS nodes
- msg: includes all custom ROS message definitions used for the data processing and filtering nodes
- worlds: defines the Gazebo simulation world and the physics engine with corresponding necessary properties to correctly simulate the Jackal robot
- config: includes the yaml file needed to adjust the publish rate of the /joint_states topic, needed to have approximately equal control input and system output update frequencies
- matlab: includes MATLAB files used to analyse the simulation and filter results
- bagfiles: includes recorded ROS topics used in post-simulation analysis in MATLAB
- doc: includes documentation to install all required software and reproduce the obtained simulation results. See below

This ROS package contains the following files:
- CMakeLists.txt: build script for the ROS package
- package.xml: contains meta-data and dependencies of the ROS package
- LICENSE: this software is licensed under Apache License 2.0


## Documentation
### installation_manual
This documents contains the instructions to install the software on an Ubuntu 16.04 operating system. The installation manual of the following software parts is included:
- ROS Kinetic
- Gazebo 7
- Gazebo 9
- Jackal packages for simulation in Gazebo
- MATLAB R2019a

Note that this manual is partly finished yet! Some installation details still need to be verified.

### system_overview
This document provides an overview of all tunable parameters in the system, sorted by system part:
- Simulation control
- Gazebo simulation
- Data processing
- Kalman filter
- Active Inference algorithm


## Acknowledgements
The jackal_gazebo package is used to start the Gazebo simulator and spawn the Jackal robot. The repository can be found [here.](https://github.com/jackal/jackal_simulator/tree/kinetic-devel/jackal_gazebo)
