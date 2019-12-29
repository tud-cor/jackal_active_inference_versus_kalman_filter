%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kalman example
%
% Example of how to use data from a ROS bag to run the Kalman filter,
% calculate its error with respect to the Gazebo data and plot the results.
% This script also provides the possibility to simulate the linear model in
% MATLAB without using the information from the ROS bag.
% 
% Author: Dennis Benders, TU Delft
% Last modified: 29.12.2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear all;
close all;
clc;


%% Load saved data to run Kalman filter on
% Retrieve operating point
load("linsys_fit_gazebo_u-y_interpolated.mat", 'mean_u', 'mean_y');

% Retrieve bag file
cd ../bagfiles;
bag = rosbag('kalman_ref_data.bag');
cd ../matlab;

% Store necessary data in arrays
bag_kalman = select(bag, 'Time', [bag.StartTime, bag.StartTime + 16], ...
    'Topic', '/filter/kalman/output');
msgs_kalman = readMessages(bag_kalman, 'DataFormat', 'Struct');
N = length(msgs_kalman);

time = zeros(1, N);
y_model = zeros(1, N);
y = zeros(1, N);
u = zeros(2, N);
result = zeros(1, N);

for i = 1:N
    time(i) = bag_kalman.MessageList.Time(i);
    y_model(i) = msgs_kalman{i}.Y(1);
    y(i) = msgs_kalman{i}.Y(3);
    u(:,i) = msgs_kalman{i}.U;
    result(i) = msgs_kalman{i}.XFilt;
end

time = time - time(1);


%% System dynamics
% Parameters
m       = 18.431;   %kg
I       = 0.5485;	%kg m²
r_wheel = 0.098;	%m
a       = 0.131;    %m
b       = 0.2078;   %m
d_long  = 227.8859853453297;    %Ns/m
d_lat   = 1102.025999065491;    %Ns/m

% System matrices
A = [-4*d_long/m, 0, 0; 0, -4*d_lat/m, 0; 0, 0, -4/I*(b^2*d_long + a^2*d_lat)];
B = [2*d_long*r_wheel/m, 2*d_long*r_wheel/m; 0, 0; 2*b*d_long*r_wheel/I, -2*b*d_long*r_wheel/I];
C = eye(3);
D = zeros(3,2);

% Discretize system
sysc = ss(A,B,C,D);
Ts = 0.001;  %1 kHz (frequency of \gazebo\model_states output)
sysd = c2d(sysc, Ts);
Gamma = sysd.A(3,3);
Theta = sysd.B(3,:);
Cd = sysd.C(3,3);


%% Call Kalman function
% Initialize state and covariance matrices
x0 = 0;
x0_hat = 0;
P0_hat = 0;
sigma_w = 0.0086532635;
Q = sigma_w^2;
sigma_z = 5*sigma_w;
R = sigma_z^2;

% Generate noise data for model simulation in MATLAB
process_noise = sigma_w*randn(1, N);
output_noise = sigma_z*randn(1, N);

% Initialize simulation arrays
x = zeros(1, N+1);
x(1) = x0;

x_hat = zeros(1, N+1);
P_hat = zeros(1, N+1);
x_hat(1) = x0_hat;
P_hat(1) = P0_hat;
for i = 1:N
    % Transform input and output to origin when applying the Kalman filter
    % with linearized system, because linearized system only explains the
    % input and output values deviations from the operating point
%     u(:,i) = mean_u;
%     x(i+1) = Gamma*x(i) + Theta*(u(:,i)-mean_u) + process_noise(i);
%     y(i) = Cd*x(i) + mean_y + output_noise(i);
    [x_hat(i+1), P_hat(i+1)] = kalman((u(:,i)-mean_u), (y(i)-mean(y)), x_hat(i), P_hat(i), Gamma, Theta, Cd, Q, R);
end

% Transform the estimate of x back to the operating point via the steady
% state value of y
x_hat = x_hat + Cd\mean_y;

% To use exactly the same data points as the code in Python generates
x_hat = x_hat(2:end);


%% Calculate MSE with respect Gazebo model
SSE = (y_model - result).^2;
MSE = sum(SSE)/N;
fprintf("MSE of Kalman filter is: %f.\n", MSE);


%% Plot data
% State estimates comparison with system output
plot(time, y);
hold on;
plot(time, result);
plot(time, x_hat);

h_title = title('State estimate result of Kalman filter');
h_xlabel = xlabel('Time (s)');
h_ylabel = ylabel('d\phi/dt (rad/s)');
h_legend = legend('System output', 'Python state estimate', 'Matlab state estimate');

set(gca,'FontSize',24);
set(h_title, 'Fontsize', 30);
set(h_xlabel, 'Fontsize', 26);
set(h_ylabel, 'Fontsize', 26);

% State estimate comparison with Gazebo model state
figure(2);
plot(time, y_model);
hold on;
plot(time, result);

h_title = title('Gazebo model state and Kalman filter state estimate');
h_xlabel = xlabel('Time (s)');
h_ylabel = ylabel('d\phi/dt (rad/s)');
h_legend = legend('Gazebo model state', 'Kalman filter state estimate');

set(gca,'FontSize',24);
set(h_title, 'Fontsize', 30);
set(h_xlabel, 'Fontsize', 26);
set(h_ylabel, 'Fontsize', 26);