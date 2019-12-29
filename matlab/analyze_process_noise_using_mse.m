%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Process noise analysis
%
% The process noise is considered to be the difference between the Gazebo
% model state and what the fitted LTI system model accounts for. This
% scripts visualizes the process noise and calculates its standard
% deviation.
% 
% Author: Dennis Benders, TU Delft
% Last modified: 29.12.2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear all;
close all;
clc;


%% Load input-output data
load linsys_fit_gazebo_u-y_interpolated.mat;


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
A = -4/I*(b^2*d_long + a^2*d_lat);
B = [2*b*d_long*r_wheel/I, -2*b*d_long*r_wheel/I];
C = 1;
D = [0, 0];

% Discretize system
sysc = ss(A,B,C,D);
Ts = 0.001;  %1 kHz (frequency of \gazebo\model_states output)
sysd = c2d(sysc, Ts);
Gamma = sysd.A;
Theta = sysd.B;


%% Determine remaining noise after linearizing the system
% Linear system is fitted to noisy data from input to output. However, the
% fit still has some error remaining. The linear system thus does not
% account for all output noise, given a certain input sequence. The
% remaining noise should thus be considered as process noise

% Calculate difference between real output signal and output accounted for
system_account = zeros(1, N_y-1);
process_noise = zeros(1, N_y-1);
for i = 1:N_y-1
    system_account(i) = Gamma*y(i) - Theta*u_int(:,i);
    process_noise(i) = y(i+1) - Gamma*y(i) - Theta*u_int(:,i);
end


%% Determine process noise properties
cov_process_noise = cov(process_noise);
sd_process_noise = sqrt(cov_process_noise);
fprintf('Variance of phi_dot Gazebo model: %.10f.\n', cov_process_noise);
fprintf('Standard deviation of phi_dot Gazebo model: %.10f.\n', sd_process_noise);


%% Plot data
subplot(3, 1, 1);
plot(time_y, y);
title("Original system output (y)");
axis([0 16 -0.066 0.066]);
xlabel("Time (s)");
ylabel("\phi dot (rad/s)");

subplot(3, 1, 2);
plot(time_y(1,1:end-1), system_account);
title("Output signal linear system accounts for");
axis([0 16 -0.065 0.065]);
xlabel("Time (s)");
ylabel("\phi dot (rad/s)");

subplot(3, 1, 3);
plot(time_y(1,1:end-1), process_noise);
title("Process noise: remaining noise");
axis([0 16 -0.065 0.065]);
xlabel("Time (s)");
ylabel("\phi dot (rad/s)");