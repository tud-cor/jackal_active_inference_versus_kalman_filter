%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LTI system model fit to Gazebo data
%
% Fitting the LTI model of the Jackal robot to the Gazebo data by
% optimising for its tunable parameters d_long and d_lat using a
% brute-force method.
% To avoid loading the ROS bag, the mat file Linsys_fit_Gazebo_u-y is
% provided with the necessary data.
% To avoid interpolating, the result is already provided in the mat file 
% Linsys_fit_Gazebo_u-y_interpolated.
% 
% Author: Dennis Benders, TU Delft
% Last modified: 29.12.2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear all;
close all;
clc;


%% Load saved data to to get input-output sequence of Gazebo simulation
% load linsys_fit_gazebo_u-y.mat;
% 
% 
% %% Cut off beginning and end of input-output data
% % Angular velocity goal of the Jackal robot: start using the recorded data
% % after y has reached this value
% % Limit y to have no time value higher than u. This causes the
% % interpolation to fail
% ang_vel = 22.5/180*pi;
% 
% for i = 1:N_y
%     if y(i) >= ang_vel
%         break;
%     end
% end
% for j = 1:N_y
%     if time_y(j) > time_u(end)
%         break;
%     end
% end
% start_time = time_y(i);
% time_y = time_y(i:j-1);
% y = y(i:j-1);
% N_y = length(time_y);
% 
% % Ensure that u has one sample earlier than the starting time (or later),
% % because we should be able to interpolate for all time stamps
% for i = 1:N_u
%     if time_u(i) >= start_time
%         break;
%     end
% end
% time_u = time_u(i-1:end);
% u = u(:,i-1:end);
% N_u = length(time_u);
% 
% % A linear system can only account for the variations of the input and
% % output signals around an operating point (around which the system is
% % linearized). Therefore, subtract the means from u and y
% mean_y = mean(y);
% y = y - mean_y;
% mean_u = [mean(u(1,:)); mean(u(2,:))];
% u(1,:) = u(1,:) - mean_u(1);
% u(2,:) = u(2,:) - mean_u(2);
% 
% 
% %% Start time at t = 0
% % Align with output, first sample input can be at negative time
% time_u = time_u - time_y(1);
% time_y = time_y - time_y(1);
% 
% 
% %% Interpolation of input (50 Hz) w.r.t. output (1 kHz)
% u_int = zeros(2,N_y);
% u_idx = 1;
% for i = 1:N_y
%     % Get next time index of u where time_u >= time_y(i)
%     while time_u(u_idx) < time_y(i)
%         u_idx = u_idx + 1;
%     end
%     
%     % If time_u == time_y: no interpolation needed; just use that sample
%     if time_u(u_idx) == time_y(i)
%         u_int(:,i) = u(:,u_idx);
%         
%     % If time_u > time_y: interpolate using time_u(u_idx-1) and time_u(u_idx)
%     else
%         u_int(:,i) = u(:,u_idx-1) + (u(:,u_idx) - u(:,u_idx-1))/(time_u(u_idx) - time_u(u_idx-1))*(time_y(i) - time_u(u_idx-1));
%     end
% end
% 
% 
load linsys_fit_gazebo_u-y_interpolated.mat;


%% System parameters (constants during iterations)
m       = 18.431;       %kg
I       = 0.5485;       %kg m²
r_wheel = 0.098;        %m
a       = 0.131;        %m
b       = 0.2078;       %m
d_long  = 0:10:1500;    %Ns/m
d_lat   = 0:10:1500;    %Ns/m

c1 = -4/I*b^2;
c2 = -4/I*a^2;
c3 = 2*b*r_wheel/I;
c4 = -2*b*r_wheel/I;


%% Calculate MSE for every (d_long, d_lat) combination
d_long_len = length(d_long);
d_lat_len = length(d_lat);
MSE = zeros(d_long_len, d_lat_len);

% Time measurement: 410 seconds needed for 150x150=22500 iterations
% => 22500/410 = 55 iteration/s
tic;
for i = 1:d_long_len
    for j = 1:d_lat_len
        
        % Discretize system
        A = c1*d_long(i) + c2*d_lat(j);
        B = [c3*d_long(i), c4*d_long(i)];
        C = 1;
        D = 0;
        sysc = ss(A,B,C,D);
        Ts = 0.001;  %1 kHz (frequency of \gazebo\model_states output)
        sysd = c2d(sysc, Ts);
        Gamma = sysd.A;
        Theta = sysd.B;

        % Calculate mean squared error + noise using discretized system
        for k = 1:N_y-1
            MSE(i,j) = MSE(i,j) + (y(k+1) - Gamma*y(k) - Theta*u_int(:,k))^2;
        end
    end
end
MSE = MSE/(N_y-1);
toc;


%% Get minimizing d_long and d_lat of MSE
% Determine indices of MSE minimum
[d_long_opt_idx, d_lat_opt_idx] = find(MSE == min(min(MSE)));

% Determine corresponding optimal values for d_long and d_lat
d_long_opt = d_long(d_long_opt_idx);
d_lat_opt = d_lat(d_lat_opt_idx);


%% Plot data
surf(d_lat, d_long, MSE);

h_title = title("MSE LTI model and Gazebo simulation data");
h_xlabel = xlabel("d_{lat}");
h_ylabel = ylabel("d_{long}");
h_zlabel = zlabel("MSE");

set(gca,'FontSize',24);
set(h_title, 'Fontsize', 30);
set(h_xlabel, 'Fontsize', 26);
set(h_ylabel, 'Fontsize', 26);
set(h_zlabel, 'Fontsize', 26);