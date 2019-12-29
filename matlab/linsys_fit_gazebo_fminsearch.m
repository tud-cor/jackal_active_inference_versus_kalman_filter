%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LTI system model fit to Gazebo data
%
% Fitting the LTI model of the Jackal robot to the Gazebo data by
% optimising for its tunable parameters d_long and d_lat using the
% fminsearch function.
% Note that the interpolated data provided by mat file 
% Linsys_fit_Gazebo_u-y_interpolated is needed to run this script.
%
% Author: Dennis Benders, TU Delft
% Last modified: 29.12.2019
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization
clear all;
close all;
clc;


%% Load interpolated input-output data
load linsys_fit_gazebo_u-y_interpolated.mat;


%% System parameters
m       = 18.431;       %kg
I       = 0.5485;       %kg m²
r_wheel = 0.098;        %m
a       = 0.131;        %m
b       = 0.2078;       %m
d_long  = 1500;
d_lat   = 1500;

c1 = -4/I*b^2;
c2 = -4/I*a^2;
c3 = 2*b*r_wheel/I;
c4 = -2*b*r_wheel/I;
d0 = [d_long, d_lat];


%% Optimise (d_long,d_lat) using fminsearch
tic;
fun = @linsys_fit_gazebo_fminsearch_objfun;
[d, fval, exitflag] = fminsearch(fun, d0);
toc;