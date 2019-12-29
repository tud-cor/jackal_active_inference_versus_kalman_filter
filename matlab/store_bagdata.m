function [topics_out] = store_bagdata(bag, topics, time)
% STORE_BAGDATA Store data of a rosbag in Matlab matrices
%
%   Author: Dennis Benders, TU Delft
%   Last edited: 17.11.2019
%
%   Input:	bagname:    string with name of bag (excluding ".bag")
%           topics:     struct with selection of the data you want to be 
%                       stored
%           time:       double array: 1st index is starting time, 2nd index 
%                       is stoptime, until maximum of the total recorded 
%                       time is hit
%
%   Output: topics_out: struct containing all (row) arrays as specified in
%                       input
%
%   For usage, see rosbag_readout_auto.m.

%--------------------------------------------------------------------------
%Check input arguments
if nargin < 3
    error("STORE_BAGDATA: please specify all 3 input arguments.");
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Check timing data
if time(1) > time(2)
    error("STORE_BAGDATA: end time must be larger than start time.");
elseif bag.EndTime < bag.StartTime + time(1)
    error("STORE_BAGDATA: start time out of bounds.");
elseif bag.EndTime < bag.StartTime + time(2)
    warning("STORE_BAGDATA: end time out of bounds.\n%s", ...
        "Your data will be cut off at the end of the available time.");
end

%Calculate total recording time to be stored in arrays
end_time = min(bag.EndTime, bag.StartTime + time(2));

%Initialize array with starting times of the different messages
start_times = zeros(1,4);

%Initialize the indication whether a certain topic has to be stored
cmd_vel = 0;
joint_states = 0;
model = 0;
imu = 0;
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Retreive information from cmd_vel if desired
if topics.cmd_vel_x_lin_vel || topics.cmd_vel_z_ang_vel
    cmd_vel = 1;
    
    %Get bag data
    bag_cmd_vel = select(bag,'Time',...
        [bag.StartTime + time(1), end_time],'Topic','/cmd_vel');
    msgs_cmd_vel = readMessages(bag_cmd_vel, 'DataFormat', 'Struct');
    len_cmd_vel = length(msgs_cmd_vel);
    
    %Store data of cmd_vel in arrays
    cmd_vel_time = zeros(1, len_cmd_vel);
    if topics.cmd_vel_x_lin_vel
        cmd_vel_x_lin_vel = zeros(1, len_cmd_vel);
    end
    if topics.cmd_vel_z_ang_vel
        cmd_vel_z_ang_vel = zeros(1, len_cmd_vel);
    end

    for i = 1:len_cmd_vel
        cmd_vel_time(i) = bag_cmd_vel.MessageList.Time(i);
        if topics.cmd_vel_x_lin_vel
            cmd_vel_x_lin_vel(i) = msgs_cmd_vel{i}.Linear.X;
        end
        if topics.cmd_vel_z_ang_vel
            cmd_vel_z_ang_vel(i) = msgs_cmd_vel{i}.Angular.Z;
        end
    end
    
    %Update start_times array
    start_times(1) = cmd_vel_time(1);
    
    %Remove unncessary data for later on in this function to save memory
    clear bag_cmd_vel msgs_cmd_vel;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Retreive information from joint_states if desired
if topics.joint_states_omega_l_f || topics.joint_states_omega_l_r || topics.joint_states_omega_r_f || topics.joint_states_omega_r_r
    joint_states = 1;
    
    %Get bag data
    bag_joint_states = select(bag,'Time',...
        [bag.StartTime + time(1), end_time],'Topic','/joint_states');
    msgs_joint_states = readMessages(bag_joint_states, 'DataFormat', 'Struct');
    len_joint_states = length(msgs_joint_states);
    
    %Store data of joint states in arrays
    joint_states_time = zeros(1, len_joint_states);
    if topics.joint_states_omega_l_f
        joint_states_omega_l_f = zeros(1, len_joint_states);
    end
    if topics.joint_states_omega_l_r
        joint_states_omega_l_r = zeros(1, len_joint_states);
    end
    if topics.joint_states_omega_r_f
        joint_states_omega_r_f = zeros(1, len_joint_states);
    end
    if topics.joint_states_omega_r_r
        joint_states_omega_r_r = zeros(1, len_joint_states);
    end
    
    for i = 1:len_joint_states
        joint_states_time(i) = bag_joint_states.MessageList.Time(i);
        if topics.joint_states_omega_l_f
            joint_states_omega_l_f(i) = msgs_joint_states{i}.Velocity(1);
        end
        if topics.joint_states_omega_l_r
            joint_states_omega_l_r(i) = msgs_joint_states{i}.Velocity(3);
        end
        if topics.joint_states_omega_r_f
            joint_states_omega_r_f(i) = msgs_joint_states{i}.Velocity(2);
        end
        if topics.joint_states_omega_r_r
            joint_states_omega_r_r(i) = msgs_joint_states{i}.Velocity(4);
        end
    end
    
    %Update start_times array
    start_times(2) = joint_states_time(1);
    
    %Remove unncessary data for later on in this function to save memory
    clear bag_joint_states msgs_joint_states;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Retreive information from model if desired
if topics.model_x_lin_vel || topics.model_y_lin_vel || topics.model_z_ang_vel
    model = 1;
    
    %Get bag data
    bag_model = select(bag,'Time',...
        [bag.StartTime + time(1), end_time],'Topic','/gazebo/model_states');
    msgs_model = readMessages(bag_model, 'DataFormat', 'Struct');
    len_model = length(msgs_model);
    
    %Store data of model in arrays
    model_time = zeros(1, len_model);
    if topics.model_x_lin_vel
        model_x_lin_vel = zeros(1, len_model);
    end
    if topics.model_y_lin_vel
        model_y_lin_vel = zeros(1, len_model);
    end
    if topics.model_z_ang_vel
        model_z_ang_vel = zeros(1, len_model);
    end
    
    for i = 1:len_model
        model_time(i) = bag_model.MessageList.Time(i);
        if topics.model_x_lin_vel
            model_x_lin_vel(i) = msgs_model{i}.Twist(2).Linear.X;
        end
        if topics.model_y_lin_vel
            model_y_lin_vel(i) = msgs_model{i}.Twist(2).Linear.Y;
        end
        if topics.model_z_ang_vel
            model_z_ang_vel(i) = msgs_model{i}.Twist(2).Angular.Z;
        end
    end
    
    %Update start_times array
    start_times(3) = model_time(1);
    
    %Remove unncessary data for later on in this function to save memory
    clear bag_model msgs_model;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Retreive information from IMU if desired
if topics.imu_x_lin_acc || topics.imu_y_lin_acc || topics.imu_z_ang_vel
    imu = 1;
    
    %Get bag data
    bag_imu = select(bag,'Time',...
        [bag.StartTime + time(1), end_time],'Topic','/imu/data');
    msgs_imu = readMessages(bag_imu, 'DataFormat', 'Struct');
    len_imu = length(msgs_imu);
    
    %Store data of IMU in arrays
    imu_time = zeros(1, len_imu);
    if topics.imu_x_lin_acc
        imu_x_lin_acc = zeros(1, len_imu);
    end
    if topics.imu_y_lin_acc
        imu_y_lin_acc = zeros(1, len_imu);
    end
    if topics.imu_z_ang_vel
        imu_z_ang_vel = zeros(1, len_imu);
    end

    for i = 1:len_imu
        imu_time(i) = bag_imu.MessageList.Time(i);
        if topics.imu_x_lin_acc
            imu_x_lin_acc(i) = msgs_imu{i}.LinearAcceleration.X;
        end
        if topics.imu_y_lin_acc
            imu_y_lin_acc(i) = msgs_imu{i}.LinearAcceleration.Y;
        end
        if topics.imu_z_ang_vel
            imu_z_ang_vel(i) = msgs_imu{i}.AngularVelocity.Z;
        end
    end
    
    %Update start_times array
    start_times(4) = imu_time(1);
    
    %Remove unncessary data for later on in this function to save memory
    clear bag_imu msgs_imu;
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Align starting times in matrices
%Determine which starting times of the topics is the highest
[max_start_time, idx_max_start_time] = max(start_times);

%Align the other matrices with this time: samples earlier than this time of
%the other topics are ignored
%Start time at 0
if idx_max_start_time == 1
    cmd_vel_time = cmd_vel_time - max_start_time;
elseif idx_max_start_time == 2
    joint_states_time = joint_states_time - max_start_time;
elseif idx_max_start_time == 3
    model_time = model_time - max_start_time;
else
    imu_time = imu_time - max_start_time;
end

%Either model or IMU has the highest starting time
%-> truncate cmd_vel
if (idx_max_start_time == 2 || idx_max_start_time == 3 ...
        || idx_max_start_time == 4) && cmd_vel
    for i = 1:len_cmd_vel
        if cmd_vel_time(i) >= max_start_time
            break;
        end
    end
    cmd_vel_time = cmd_vel_time(:, i:end);
    cmd_vel_time = cmd_vel_time - max_start_time;
    if topics.cmd_vel_x_lin_vel
        cmd_vel_x_lin_vel = cmd_vel_x_lin_vel(:, i:end);
    end
    if topics.cmd_vel_z_ang_vel
        cmd_vel_z_ang_vel = cmd_vel_z_ang_vel(:, i:end);
    end
end

%Either cmd_vel, model or IMU has the highest starting time
%-> truncate joint_states
if (idx_max_start_time == 1 || idx_max_start_time == 3 ...
        || idx_max_start_time == 4) && joint_states
    for i = 1:len_joint_states
        if joint_states_time(i) >= max_start_time
            break;
        end
    end
    joint_states_time = joint_states_time(:, i:end);
    joint_states_time = joint_states_time - max_start_time;
    if topics.joint_states_omega_l_f
        joint_states_omega_l_f = joint_states_omega_l_f(:, i:end);
    end
    if topics.joint_states_omega_l_r
        joint_states_omega_l_r = joint_states_omega_l_r(:, i:end);
    end
    if topics.joint_states_omega_r_f
        joint_states_omega_r_f = joint_states_omega_r_f(:, i:end);
    end
    if topics.joint_states_omega_r_r
        joint_states_omega_r_r = joint_states_omega_r_r(:, i:end);
    end
end

%Either cmd_vel, joint_states or IMU has the highest starting time
%-> truncate model
if (idx_max_start_time == 1 || idx_max_start_time == 2 ...
        || idx_max_start_time == 4) && model
    for i = 1:len_model
        if model_time(i) >= max_start_time
            break;
        end
    end
    model_time = model_time(:, i:end);
    model_time = model_time - max_start_time;
    if topics.model_x_lin_vel
        model_x_lin_vel = model_x_lin_vel(:, i:end);
    end
    if topics.model_y_lin_vel
        model_y_lin_vel = model_y_lin_vel(:, i:end);
    end
    if topics.model_z_ang_vel
        model_z_ang_vel = model_z_ang_vel(:, i:end);
    end
end

%Either cmd_vel, joint_states or model has the highest starting time
%-> truncate IMU
if (idx_max_start_time == 1 || idx_max_start_time == 2 ...
        || idx_max_start_time == 3) && imu
    for i = 1:len_imu
        if imu_time(i) >= max_start_time
            break;
        end
    end
    imu_time = imu_time(1, i:end);
    imu_time = imu_time - max_start_time;
    if topics.imu_x_lin_acc
        imu_x_lin_acc = imu_x_lin_acc(:, i:end);
    end
    if topics.imu_y_lin_acc
        imu_y_lin_acc = imu_y_lin_acc(:, i:end);
    end
    if topics.imu_z_ang_vel
        imu_z_ang_vel = imu_z_ang_vel(:, i:end);
    end
end
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%Create function output
if cmd_vel
    topics_out.cmd_vel_time = cmd_vel_time;
    if topics.cmd_vel_x_lin_vel
        topics_out.cmd_vel_x_lin_vel = cmd_vel_x_lin_vel;
    end
    if topics.cmd_vel_z_ang_vel
        topics_out.cmd_vel_z_ang_vel = cmd_vel_z_ang_vel;
    end
end

if joint_states
    topics_out.joint_states_time = joint_states_time;
    if topics.joint_states_omega_l_f
        topics_out.joint_states_omega_l_f = joint_states_omega_l_f;
    end
    if topics.joint_states_omega_l_r
        topics_out.joint_states_omega_l_r = joint_states_omega_l_r;
    end
    if topics.joint_states_omega_r_f
        topics_out.joint_states_omega_r_f = joint_states_omega_r_f;
    end
    if topics.joint_states_omega_r_r
        topics_out.joint_states_omega_r_r = joint_states_omega_r_r;
    end
end

if model
    topics_out.model_time = model_time;
    if topics.model_x_lin_vel
        topics_out.model_x_lin_vel = model_x_lin_vel;
    end
    if topics.model_y_lin_vel
        topics_out.model_y_lin_vel = model_y_lin_vel;
    end
    if topics.model_z_ang_vel
        topics_out.model_z_ang_vel = model_z_ang_vel;
    end
end

if imu
    topics_out.imu_time = imu_time;
    if topics.imu_x_lin_acc
        topics_out.imu_x_lin_acc = imu_x_lin_acc;
    end
    if topics.imu_y_lin_acc
        topics_out.imu_y_lin_acc = imu_y_lin_acc;
    end
    if topics.imu_z_ang_vel
        topics_out.imu_z_ang_vel = imu_z_ang_vel;
    end
end
%--------------------------------------------------------------------------
end