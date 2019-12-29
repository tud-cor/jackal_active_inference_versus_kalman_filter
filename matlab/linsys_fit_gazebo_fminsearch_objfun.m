function MSE = linsys_fit_gazebo_fminsearch_objfun(d)    
% LINSYS_FIT_GAZEBO_FMINSEARCH_OBJFUN  Execute fminsearch to optimize
% d_long and d_lat
%
%   Author: Dennis Benders, TU Delft
%   Last edited: 29.12.2019
%
%   Input:  d:      array containing current d_long and d_lat values
%
%   Output: MSE:    MSE value for current combination of d_long and d_lat
%
%   For usage, see linsys_fit_gazebo_fminsearch.m.

% Load interpolated input-output data
load linsys_fit_gazebo_u-y_interpolated.mat;

% Discretize system
c1 = -0.314901294439380;
c2 = -0.125148587055606;
c3 = 0.074254876937101;
c4 = -0.074254876937101;
A = c1*d(1) + c2*d(2);
B = [c3*d(1), c4*d(1)];
C = 1;
D = 0;
sysc = ss(A,B,C,D);
Ts = 0.001;  %1 kHz (frequency of \gazebo\model_states output)
sysd = c2d(sysc, Ts);
Gamma = sysd.A;
Theta = sysd.B;

% Calculate mean squared error + noise using discretized system
MSE = 0;
for k = 1:N_y-1
    MSE = MSE + (y(k+1) - Gamma*y(k) - Theta*u_int(:,k))^2;
end
MSE = MSE/(N_y-1);
end