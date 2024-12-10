%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RBE_501_Task_1.m
%
% Input: None, positions are hardcoded
%
% Output: Robot moves to home, then A, then B
%
% Written by Sebastian Baldini & Brighton Lee
% 12/1/2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clc;
clear;

%% SETUP
% Input speed of A to B movement in seconds (used for task 1 data
% collection)
run_time = 1000/0.4;

% Set up robot and go home
robot = Robot();
cal_home = [0,0,0,0];

% Define transformations from A and B
phiA = -pi/4;
phiB = atan2(0.17,0.185);
T_A = [[0,-sin(phiA),-cos(phiA),0.185];
    [0,cos(phiA),-sin(phiA),-0.185];
    [1,0,0,0.185];
    [0,0,0,1]];
T_B = [[0,-sin(phiB),-cos(phiB),0.185];
    [0,cos(phiB),-sin(phiB),0.170];
    [1,0,0,0.07];
    [0,0,0,1]];

% Get joint angles for A and B
[theta_A,s] = robot.ikspace(T_A);
[theta_B,s] = robot.ikspace(T_B);


%% MOVE ROBOT
% Move to home, then A, then B
pause(1);
robot.measured_js(1,0,0)
robot.interpolate_jp(cal_home, 2000) % position, time in milliseconds
robot.measured_js(1,0,0)
robot.interpolate_jp(rad2deg(theta_A), 2000)
pause(1)
robot.measured_js(1,0,0)
[current_arr,t] = robot.interpolate_jp_record(rad2deg(theta_B), run_time);
pause(1);

%% PLOT CURRENTS
figure

subplot(2, 2, 1);
plot(t,current_arr(:,1));
axis([t(1) t(end) -250 250])
xlabel('Time (s)')
ylabel('Current (mA)')
title('Joint 1 Current Draw');

subplot(2, 2, 2);
plot(t,current_arr(:,2));
axis([t(1) t(end) -250 250])
xlabel('Time (s)')
ylabel('Current (mA)')
title('Joint 2 Current Draw');

subplot(2, 2, 3);
plot(t,current_arr(:,3));
axis([t(1) t(end) -250 250])
xlabel('Time (s)')
ylabel('Current (mA)')
title('Joint 3 Current Draw');

subplot(2, 2, 4);
plot(t,current_arr(:,4));
axis([t(1) t(end) -250 250])
xlabel('Time (s)')
ylabel('Current (mA)')
title('Joint 4 Current Draw');