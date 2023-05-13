%% PIR Linear System Initialization
% PIR = Perturb, Identify and Reconfigure
% This script initializes the "PIR_IPoC.slx" Simulink Model
% Author: Benjamin Kelm, 13.05.2023

%% Code Overview
% * Simulates discrete linear System -> Time-Series Data 
% * Performs online sparse regression on custom basis (state's timeseries) via
% SINDy algorithm.
% * Identifies up-to-date State-Space (SS) model and computes a Full-State
% Feedback Controller K via LQR approach

% Prepare Directory
close all
clear all


%% Generate Linear State-Space Model

%% Pendulum on a Cart
m = 0.05;  % Mass of pendulum up mass 
M = 0.20;  % Mass of Cart 
L = 0.2;  % Length of Stick
g = -10; % Gravity 
d = 1; % Dampening
b = 1; % Pendulum up (b=1), Pendulum DOWN (b=-1)

A = [   -d/M  b*m*g/M          0; % v [m/s] - Velocity of Cart
            0        0          1; % Theta [rad] - Pole angle
     -b*d/(M*L) -b*(m+M)*g/(M*L)  0] % omega [rad/s] - Pole angular velocity

% Actuator Model: PT1 System
act.k1 = 1; % [N/u_in] Control effectiveness Constants

B = [act.k1*1/M % Force on Cart causes acceleration
    0
    act.k1*b*1/(M*L)]  % Force on Cart causes angular acceleration


% System Analysis - Check Stability
fprintf('Eigenvalue %2.2f %2.2f %2.2f %2.2f\n', eig(A))

dim_u = size(B,2); % input dimensions
dim_x = size(A,2); % dimensions in state-space

C = eye(dim_x);
D = zeros(dim_x,dim_u);

lS = ss(A,B,C,D); % Generate continuous State-Space Model

%% Specify Inputs for Simulation
t_end = 30; % [s] - Simulation End Time
T_sim = 0.005; % [s] - Sim. Sample Time

x0 = [0, pi, 0]'; % Initial State

%% Transform model to Discrete State Space
lS_d = c2d(lS, T_sim,'zoh');

%% Develop preliminary FSF Controller
Q = eye(dim_x) .* [2 5 1];
R = eye(dim_u) .* [0.01];
K_lqr = lqrd(A, B, Q, R, T_sim) % Preliminary FSF Controller for non-degraded Plant
% K_pre = dlqr(lS_d.A, lS_d.B, Q, R) % Preliminary FSF Controller for non-degraded Plant

%% Simulation Control
