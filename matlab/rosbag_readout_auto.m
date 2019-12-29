%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Automatic readout of ROS bags and storing data in MATLAB arrays
%
% Script used to call the store_bagdata function in order to select the
% messages from the ROS bags that are desired. The topics covered are:
% /cmd_vel, /joint_states, /gazebo/model_states and /imu/data.
% 
% Author: Dennis Benders, TU Delft
% Last modified: 17.11.2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear all;
close all;
clc;


%% Set variables
% Retrieve bag file
cd ../bagfiles;
bag = rosbag("test_store_bagdata.bag");
cd ../matlab;

topics.cmd_vel_x_lin_vel = 0;
topics.cmd_vel_z_ang_vel = 1;

topics.joint_states_omega_l_f = 1;
topics.joint_states_omega_l_r = 1;
topics.joint_states_omega_r_f = 1;
topics.joint_states_omega_r_r = 1;

topics.model_x_lin_vel = 0;
topics.model_y_lin_vel = 0;
topics.model_z_ang_vel = 1;

topics.imu_x_lin_acc = 0;
topics.imu_y_lin_acc = 0;
topics.imu_z_ang_vel = 1;

time = [0, 16];


%% Get data
topics_out = store_bagdata(bag, topics, time);

if topics.cmd_vel_x_lin_vel || topics.cmd_vel_z_ang_vel
    cmd_vel_time = topics_out.cmd_vel_time;
end
if topics.cmd_vel_x_lin_vel
    cmd_vel_x_lin_vel = topics_out.cmd_vel_x_lin_vel;
end
if topics.cmd_vel_z_ang_vel
    cmd_vel_z_ang_vel = topics_out.cmd_vel_z_ang_vel;
end

if topics.joint_states_omega_l_f || topics.joint_states_omega_l_r || topics.joint_states_omega_r_f || topics.joint_states_omega_r_r
    joint_states_time = topics_out.joint_states_time;
end
if topics.joint_states_omega_l_f
    joint_states_omega_l_f = topics_out.joint_states_omega_l_f;
end
if topics.joint_states_omega_l_r
    joint_states_omega_l_r = topics_out.joint_states_omega_l_r;
end
if topics.joint_states_omega_r_f
    joint_states_omega_r_f = topics_out.joint_states_omega_r_f;
end
if topics.joint_states_omega_r_r
    joint_states_omega_r_r = topics_out.joint_states_omega_r_r;
end

if topics.model_x_lin_vel || topics.model_y_lin_vel || topics.model_z_ang_vel
    model_time = topics_out.model_time;
end
if topics.model_x_lin_vel
    model_x_lin_vel = topics_out.model_x_lin_vel;
end
if topics.model_y_lin_vel
    model_y_lin_vel = topics_out.model_y_lin_vel;
end
if topics.model_z_ang_vel
    model_z_ang_vel = topics_out.model_z_ang_vel;
end

if topics.imu_x_lin_acc || topics.imu_y_lin_acc || topics.imu_z_ang_vel
    imu_time = topics_out.imu_time;
end
if topics.imu_x_lin_acc
    imu_x_lin_acc = topics_out.imu_x_lin_acc;
end
if topics.imu_y_lin_acc
    imu_y_lin_acc = topics_out.imu_y_lin_acc;
end
if topics.imu_z_ang_vel
    imu_z_ang_vel = topics_out.imu_z_ang_vel;
end