%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RBE_501_Task_4.m
%
% DESCRIPTION: Uses inverse dynamics to calculate the theoretical torque
% required to hold the robot at a given position, then move the robot to
% that position, read the current draw of each motor, and use that to
% calculate the theoretical torque constants of the motors
%
% INPUT: Position
%
% OUTPUT: Theoretical joint torques, approximate torque constants
%
% Written by Sebastian Baldini & Brighton Lee & Manoj Velmurugan
% 12/10/2024
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clc;
clear;

%% SETUP
% Input joint angles relative to home configuration (column)
thetalist = [0;0;0;0]; % rad

% Input joint velocities and accelerations
dthetalist = [0; 0; 0; 0]; % rad/s
ddthetalist = [0; 0; 0; 0]; % rad/s^2

% Define variables
l1 = 0.096326; % m
l2 = 0.128;
l3 = 0.024;
l4 = 0.124;
l5 = 0.1334;
g = [0; 0; -9.8]; % m/s^2

% Initialize Robot
robot = Robot();
robot.writeMode('p');

% Input home configuration (rows)
M = [[0, 0, -1, l3+l4+l5]; [0, 1, 0, 0]; [1, 0, 0, l1+l2]; [0, 0, 0, 1]];

% Input Mlist
M01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.0354]; [0, 0, 0, 1]];
M12 = [[1, 0, 0, 0.003809]; [0, 1, 0, 0]; [0, 0 ,1, 0.163326]; [0, 0, 0, 1]];
M23 = [[1, 0, 0, 0.11038]; [0, 1, 0, 0]; [0, 0, 1, 0.02676]; [0, 0, 0, 1]];
M34 = [[1, 0, 0, 0.10038]; [0, 1, 0, 0]; [0, 0, 1, 0.003165]; [0, 0, 0, 1]];
M45 = [[0, 0, 1, 0.06683]; [0, 1, 0, 0]; [-1, 0, 0, -0.003165]; [0, 0, 0, 1]];
Mlist = cat(3, M01, M12, M23, M34, M45);

% Input Glist
Ixx1 = 0.00005359; % kg*m^2
Iyy1 = 0.00005149;
Izz1 = 0.00002604;
Ixy1 = 0;
Iyz1 = 0;
Ixz1 = 0;
m1 = 0.11584;

Ixx2 = 0.00022943;
Iyy2 = 0.00023236;
Izz2 = 0.00005372;
Ixy2 = 0;
Iyz2 = 0;
Ixz2 = 0.00002833;
m2 = 0.143;

Ixx3 = 0.00002916;
Iyy3 = 0.00013968;
Izz3 = 0.00015097;
Ixy3 = 0;
Iyz3 = 0;
Ixz3 = 0;
m3 = 0.122;

Ixx4 = 0.00029155;
Iyy4 = 0.00020877;
Izz4 = 0.00019077;
Ixy4 = 0;
Iyz4 = 0;
Ixz4 = 0;
m4 = 0.225;

G1 =  [[Ixx1, Ixy1, Ixz1, 0, 0, 0]; [Ixy1, Iyy1, Iyz1, 0, 0, 0]; ...
    [Ixz1, Iyz1, Izz1, 0, 0, 0]; [0, 0, 0, m1, 0, 0]; ...
    [0, 0, 0, 0, m1, 0]; [0, 0, 0, 0, 0, m1]];
G2 =  [[Ixx2, Ixy2, Ixz2, 0, 0, 0]; [Ixy2, Iyy2, Iyz2, 0, 0, 0]; ...
    [Ixz2, Iyz2, Izz2, 0, 0, 0]; [0, 0, 0, m2, 0, 0]; ...
    [0, 0, 0, 0, m2, 0]; [0, 0, 0, 0, 0, m2]];
G3 =  [[Ixx3, Ixy3, Ixz3, 0, 0, 0]; [Ixy3, Iyy3, Iyz3, 0, 0, 0]; ...
    [Ixz3, Iyz3, Izz3, 0, 0, 0]; [0, 0, 0, m3, 0, 0]; ...
    [0, 0, 0, 0, m3, 0]; [0, 0, 0, 0, 0, m3]];
G4 =  [[Ixx4, Ixy4, Ixz4, 0, 0, 0]; [Ixy4, Iyy4, Iyz4, 0, 0, 0]; ...
    [Ixz4, Iyz4, Izz4, 0, 0, 0]; [0, 0, 0, m4, 0, 0]; ...
    [0, 0, 0, 0, m4, 0]; [0, 0, 0, 0, 0, m4]];
Glist = cat(3, G1, G2, G3, G4);

% Input wrench at end effector
Ftip = [0; 0; 0; 0; 1; 0]; % N*m, N

%% CALCULATE
% Compute joint torques
taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, ...
                        Ftip, Mlist, Glist, robot.Slist) % N*m

% Move robot to position of calculated torques
robot.interpolate_jp(thetalist,2000);

% Read joint currents
pause(2)
j_cur = robot.measured_js(0,0,1)./1000; % A
pause(1)

% Calculate approximate torque constants for each motor
torque_constants = taulist.'./j_cur(3,:) % N*m/A

