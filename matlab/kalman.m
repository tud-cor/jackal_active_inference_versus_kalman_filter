function [x_new, P_new] = kalman(u, y, x_old, P_old, A, B, C, Q, R)
% KALMAN  Execute Kalman filter
%
%   Author: Dennis Benders, TU Delft
%   Last edited: 29.12.2019
%
%   Input:  u:      current system control input
%           y:      current system output
%           x_old:  previous state estimate
%           P_old:  previous error covariance matrix
%           A:      state matrix
%           B:      input matrix
%           C:      output matrix
%           Q:      process noise covariance matrix
%           R:      output noise covariance matrix
%
%   Output: x_new:	current state estimate
%           P_new:  current error covariance matrix
%
%   For usage, see kalman_control.m.

% Prediction stage
x_pred = A*x_old + B*u;
P_pred = A*P_old*A' + Q;

% Update stage
y_tilde = y - C*x_pred;
S = C*P_pred*C' + R;
K = P_pred*C'/S;
x_new = x_pred + K*y_tilde;
P_new = (1 - K*C)*P_pred;
end