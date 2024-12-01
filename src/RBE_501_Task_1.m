%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RBE_501_Task_1.m
%
% Input: None, positions are hardcoded
%
% Output: Robot moves to home, then A, then B
%
% Written by Sebastian Baldini & Brighton Lee
% 12/1/2024
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all;
clc;
clear;

% Set up robot and gclose all;
clear;
clc;

% Set up robot and go home
robot = Robot();
cal_home = [0,0,0,0];


%% Task 1
% Define transformations from A and B
phiA = -pi/4;
phiB = atan2(0.17,0.185);
T_A = [[0,-sin(phiA),-cos(phiA),0.185];
    [0,cos(phiA),-sin(phiA),-0.185];
    [1,0,0,0.185];
    [0,0,0,1]]
T_B = [[0,-sin(phiB),-cos(phiB),0.185];
    [0,cos(phiB),-sin(phiB),0.170];
    [1,0,0,0.07];
    [0,0,0,1]]

% Get joint angles for A and B
[theta_A,s] = robot.ikspace(T_A)
[theta_B,s] = robot.ikspace(T_B)

% Move to home, then A, then B
pause(1);
robot.measured_js(1,0,0)
robot.interpolate_jp(cal_home, 2000) % position, time in milliseconds
pause(4);
robot.measured_js(1,0,0)
robot.interpolate_jp(rad2deg(theta_A), 2000)
pause(4);
robot.measured_js(1,0,0)
robot.interpolate_jp(rad2deg(theta_B), 500)
pause(1);