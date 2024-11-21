close all;
clear;
clc;

% Set up robot and go home
robot = Robot();
cal_home = [0,0,0,0];

robot.measured_js(1,0)
robot.interpolate_jp(cal_home, 2500)
pause(0.5);

robot.measured_js(1,1)
% robot.interpolate_jp([10,0,0,0], 750);
% pause(0.5);
% robot.interpolate_jp([-10,0,0,0], 750);
% pause(0.5);
