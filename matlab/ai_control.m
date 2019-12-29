%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AI example
%
% Example of how to use data from a ROS bag to run the AI algorithm on and
% plot the results.
% Note that only the filtering part of the AI algorithm is implemented yet!
% 
% Author: Dennis Benders, TU Delft
% Last modified: 29.12.2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear all;
close all;
clc;


%% Load saved data to run AI algorithm on
% Retrieve operating point
load("linsys_fit_gazebo_u-y_interpolated.mat", 'mean_u', 'mean_y');

% Retrieve bag file
cd ../bagfiles;
bag = rosbag('kalman_ref_data.bag');
cd ../matlab;

% Store necessary data in arrays
bag_ai = select(bag, 'Time', [bag.StartTime, bag.StartTime + 16], ...
    'Topic', '/filter/kalman/output');
msgs_ai = readMessages(bag_ai, 'DataFormat', 'Struct');
N = length(msgs_ai);

time = zeros(1, N);
y = zeros(1, N);
% u = zeros(2, N);
% result = zeros(1, N);

for i = 1:N
    time(i) = bag_ai.MessageList.Time(i);
    y(i) = msgs_ai{i}.Y(3);
%     u(:,i) = msgs_ai{i}.U;
%     result(i) = msgs_ai{i}.XFilt;
end

time = time - time(1);


%% System dynamics
n_states = 1;
n_inputs = 2;
n_outputs = 1;
p = 6;
delta_t = 0.001;
if n_states == 3 || n_outputs == 3
    printf("Check the AI_control code for being able to handle 3 states/outputs");
end

% Parameters
m       = 18.431;   %kg
I       = 0.5485;	%kg m²
r_wheel = 0.098;	%m
a       = 0.131;    %m
b       = 0.2078;   %m
d_long  = 227.8859853453297;    %Ns/m
d_lat   = 1102.025999065491;    %Ns/m

% System matrices
if n_states == 1
    A = -4/I*(b^2*d_long + a^2*d_lat);
    B = [2*b*d_long*r_wheel/I, -2*b*d_long*r_wheel/I];
    C = 1;
    D = zeros(1,2);
elseif n_states == 3  
    A = [-4*d_long/m, 0, 0; 0, -4*d_lat/m, 0; 0, 0, -4/I*(b^2*d_long + a^2*d_lat)];
    B = [2*d_long*r_wheel/m, 2*d_long*r_wheel/m; 0, 0; 2*b*d_long*r_wheel/I, -2*b*d_long*r_wheel/I];
    C = eye(3);
    D = zeros(3,2);
end

% Discretize system
sysc = ss(A,B,C,D);
Ts = 0.001;  %1 kHz (frequency of \gazebo\model_states output)
sysd = c2d(sysc, Ts);
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
A_tilde = kron(eye(1+p), A);
C_tilde = kron(eye(1+p), C);


%% Call AI function
% Initialize AI-specific vectors and matrices
mu0 = zeros((1+p)*n_states, 1);
Der = kron(diag(ones(p, 1), 1), eye(n_states));
alpha_mu = 3.408e-6;
alpha_u = 0.01;
mu_ref = zeros((1+p)*n_states, 1);
mu_ref(1,1) = 0.0;
xi = Der*mu_ref - A_tilde*mu_ref;
G = -1*C/A*B;

% Construct PI_w and PI_z
sigma_w = 0.0086532635;
SIGMA_w = eye(n_states)*sigma_w^2;
s_w = 0;
PI_w = generate_pi(1+p, SIGMA_w, s_w);
% PI_w = zeros((1+p)*n_states, (1+p)*n_states);
% PI_w(1,1) = 0.01;

sigma_z = 5*sigma_w;
SIGMA_z = eye(n_outputs)*sigma_z^2;
s_z = 0.1;
PI_z = generate_pi(1+p, SIGMA_z, s_z);

% Initialize simulation arrays
mu = zeros((1+p)*n_states, N+1);
mu(1,1) = -mean_y;
u_ai = zeros(n_inputs, N);

y_op = zeros(n_outputs, N); %output transformed to system operating point (where linear system is valid)
y_gen = zeros((1+p)*n_outputs, N); %generalized y
for i = 1:N
    y_op(:,i) = y(:,i) - mean_y;
    y_gen(1:n_outputs, i) = y_op(:,i); %update generalized y with current system output
    [mu(:,i+1), u_ai(:,i+1)] = ai(mu(:,i), u_ai(:,i), y_gen(:,i), delta_t, Der, alpha_mu, alpha_u, A_tilde, C_tilde, PI_w, PI_z, xi, G);
end

% Transform the estimate of x back to the operating point via the steady
% state value of y
x_hat = mu(n_states,:);
x_hat = x_hat + Cd\mean_y;

% To use exactly the same data points as the code in Python generates
x_hat = x_hat(:,2:end);


%% Plot data
plot(time, y);
hold on;
plot(time, x_hat);

h_title = title('First state estimate result of AI filter');
h_xlabel = xlabel('Time (s)');
h_ylabel = ylabel('d\phi/dt (rad/s)');
h_legend = legend('System output', 'State estimate', 'location', 'best');

set(gca,'FontSize',24);
set(h_title, 'Fontsize', 30);
set(h_xlabel, 'Fontsize', 26);
set(h_ylabel, 'Fontsize', 26);