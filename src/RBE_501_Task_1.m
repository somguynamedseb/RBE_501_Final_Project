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
pt_A = [[0,0,1,0.185];
    [0,1,0,-0.185];
    [1,0,0,0.185];
    [0,0,0,1]]
pt_B = [[0,0,1,0.185];
    [0,1,0,0.170];
    [1,0,0,0.07];
    [0,0,0,1]]

[theta_A,s] = robot.ikspace(pt_A)
[theta_B,s] = robot.ikspace(pt_B)
% pause(1);
% robot.measured_js(1,0)
% robot.interpolate_jp(cal_home, 2000)
% pause(4);
% robot.measured_js(1,0)
% robot.interpolate_jp(rad2deg(theta_A), 2000)
% pause(4);
% robot.measured_js(1,0)
% robot.interpolate_jp(rad2deg(theta_B), 2000)
% pause(1);


o home
robot = Robot();
cal_home = [0,0,0,0];


%% Task 1

PhiA = -pi/4;
PhiB = atan(0.17,0.185);

pt_A = [[0,-sin(PhiA),-cos(PhiA),0.185];
    [0,cos(PhiA),-sin(PhiA),-0.185];
    [1,0,0,0.185];
    [0,0,0,1]];


pt_B = [[0,-sin(PhiB),-cos(PhiB),0.185];
    [0,cos(PhiN),-sin(PhiB),0.17];
    [1,0,0,0.07];
    [0,0,0,1]];

[theta_A,s] = robot.ikspace(pt_A)
[theta_B,s] = robot.ikspace(pt_B)
pause(1);
robot.measured_js(1,0)
robot.interpolate_jp(cal_home, 2000)
% pause(4);
% robot.measured_js(1,0)
% robot.interpolate_jp(rad2deg(theta_A), 2000)
% pause(4);
% robot.measured_js(1,0)
% robot.interpolate_jp(rad2deg(theta_B), 2000)
% pause(1);


