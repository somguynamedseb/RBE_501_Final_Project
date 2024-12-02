clear all
clc
close all

% % Define Constants
% target_time = 10;
% tar_A = [0.185;-0.185;0.185];
% tar_B = [0.185;0.170;0.70];
% tar_C = [0.185;0;0.240];
% 
% % Initialize
robot = Robot();
% 
% % Generate Path
% [pos_arr1, vel_arr1, acc_arr1] = robot.LSPB(tar_A, tar_C, target_time);
% [pos_arr2, vel_arr2, acc_arr2] = robot.LSPB(tar_C, tar_B, target_time);
% 
% [velocity_input_1] = robot.LSPBtoVel(pos_arr1,vel_arr1);
% [velocity_input_2] = robot.LSPBtoVel(pos_arr2,vel_arr2);
% 
% % Move the robot
% cal_home = [0,0,0,0];
vels = [0.1;0;0;0]';
% pause(1);
% robot.measured_js(1,0,0);
% robot.interpolate_jp(cal_home, 2000);
% 
% pause(3);
% 
% robot.writeMode('velocity')

pause(3);
robot.writeVelocities(vels)