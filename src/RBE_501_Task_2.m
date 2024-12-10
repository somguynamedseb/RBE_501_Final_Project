%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RBE_501_Task_2.m
%
% DESCRIPTION: Uses LSPB to generate a constant velocity curve with ramp up
% and ramp down between 2 points (A, B) with a waypoint in between (C).
% Moves the robot (Open Manipulator-X) between those two points.
%
% INPUT: The 3 target points
%
% OUTPUT: Position, velocity, and acceleration curves for each movement,
% and moving the robot
%
% Written by: Sebastian Baldini, Manoj Velmurugan, Brighton Lee
% 12/9/2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
clc
close all

%% SETUP
% Define Constants
target_time = 10;
tar_A = [0.185;-0.185;0.185];
tar_B = [0.185;0.170;0.070];
tar_C = [0.185;0;0.24];
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
T_C = [[0,0,-1,0.185];
    [0,1,0,0];
    [1,0,0,0.24];
    [0,0,0,1]];


% Initialize
robot = Robot();

%% GENERATE PATH
[ee_pos_arr1, ee_vel_arr1, ee_acc_arr1] = robot.LSPB(tar_A, tar_C, target_time,2);
[ee_pos_arr2, ee_vel_arr2, ee_acc_arr2] = robot.LSPB(tar_C, tar_B, target_time,1);

[pos_arr1, vel_arr1, acc_arr1] = robot.LSPBUpdated(T_A,T_C,target_time,2);
[pos_arr2, vel_arr2, acc_arr2] = robot.LSPBUpdated(T_C,T_B,target_time,1);

%% MOVE THE ROBOT
% Move robot to point A
[A,s] = robot.ikspace(T_A);
robot.measured_js(1,0,0);
robot.interpolate_jp(rad2deg(A), 2000);

% Change to velocity control
robot.writeMode('v')
pause(3);

% Move from A to C
for i = 1:size(vel_arr1)
    % Converts to deg/s
    joint_velocity_command_1 = rad2deg(vel_arr1(i, :));
    disp(joint_velocity_command_1(1))

    % Move
    robot.writeVelocities(joint_velocity_command_1)
    pause(robot.dt)
end
pause(robot.dt)

% Move from C to B
for i = 1:size(vel_arr2)
    % Convert to deg/s
    joint_velocity_command_2 = rad2deg(vel_arr2(i, :));
    disp(joint_velocity_command_2(1))

    % Move
    robot.writeVelocities(joint_velocity_command_2)
    pause(robot.dt)
end

% Stop
robot.writeVelocities([0,0,0,0])
pause(2)

%% PLOT
figure;

ee_pos_arr = [ee_pos_arr1; ee_pos_arr2]; 
ee_vel_arr = [ee_vel_arr1; ee_vel_arr2]; 
ee_acc_arr = [ee_acc_arr1; ee_acc_arr2]; 

% Plot position data (first row)
subplot(3, 3, 1);
plot(ee_pos_arr(:, 1),'LineWidth',2.0);
axis([0 length(ee_pos_arr) 0 0.25])
xlabel('Step')
ylabel('Position (m)')
title('End Effector Position X');

subplot(3, 3, 2);
plot(ee_pos_arr(:, 2),'LineWidth',2.0);
axis([0 length(ee_pos_arr) -0.25 0.25])
xlabel('Step')
ylabel('Position (m)')
title('End Effector Position Y');

subplot(3, 3, 3);
plot(ee_pos_arr(:, 3),'LineWidth',2.0);
axis([0 length(ee_pos_arr) 0 0.25])
xlabel('Step')
ylabel('Position (m)')
title('End Effector Position Z');

% Plot velocity data (second row)
subplot(3, 3, 4);
plot(ee_vel_arr(:, 1),'LineWidth',2.0);
axis([0 length(ee_vel_arr) -0.025 0.025])
xlabel('Step')
ylabel('Velocity (m/s)')
title('End Effector Velocity X');

subplot(3, 3, 5);
plot(ee_vel_arr(:, 2),'LineWidth',2.0);
axis([0 length(ee_vel_arr) -0.025 0.025])
xlabel('Step')
ylabel('Velocity (m/s)')
title('End Effector Velocity Y');

subplot(3, 3, 6);
plot(ee_vel_arr(:, 3),'LineWidth',2.0);
axis([0 length(ee_vel_arr) -0.025 0.025])
xlabel('Step')
ylabel('Velocity (m/s)')
title('End Effector Velocity Z');

% Plot acceleration data (third row)
subplot(3, 3, 7);
plot(ee_acc_arr(:, 1),'LineWidth',2.0);
axis([0 length(ee_acc_arr) -0.015 0.015])
xlabel('Step')
ylabel('Acceleration (m/s^2)')
title('End Effector Acceleration X');

subplot(3, 3, 8);
plot(ee_acc_arr(:, 2),'LineWidth',2.0);
axis([0 length(ee_acc_arr) -0.015 0.015])
xlabel('Step')
ylabel('Acceleration (m/s^2)')
title('End Effector Acceleration Y');

subplot(3, 3, 9);
plot(ee_acc_arr(:, 3),'LineWidth',2.0);
axis([0 length(ee_acc_arr) -0.015 0.015])
xlabel('Step')
ylabel('Acceleration (m/s^2)')
title('End Effector Acceleration Z');