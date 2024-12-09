clear all
clc
close all

% Define Constants
target_time = 1;
tar_A = [0.185;-0.185;0.185];
tar_B = [0.185;0.170;0.070];
tar_C = [0.185;0;0.24];


% Initialize
robot = Robot();
s
% Generate Path
[pos_arr1, vel_arr1, acc_arr1,z_arr1] = robot.LSPB(tar_A, tar_C, target_time);
[pos_arr2, vel_arr2, acc_arr2,z_arr2] = robot.LSPB(tar_C, tar_B, target_time);

[velocity_input_1] = robot.LSPBtoVel(pos_arr1,vel_arr1,z_arr1);
[velocity_input_2] = robot.LSPBtoVel(pos_arr2,vel_arr2);

% Move the robot
phiA = -pi/4;
T_A = [[0,-sin(phiA),-cos(phiA),0.185];
    [0,cos(phiA),-sin(phiA),-0.185];
    [1,0,0,0.185];
    [0,0,0,1]];
[A,s] = robot.ikspace(T_A);
vels = [1;0;0;0];

robot.measured_js(1,0,0);
robot.interpolate_jp(rad2deg(A), 2000);

robot.writeMode('v')

pause(3);

for i = 1:size(velocity_input_1)
    joint_velocity_command = velocity_input_1(i, :)*180/pi;
    % joint_velocity_command(2) = 0;
    % joint_velocity_command(3) = 0;
    % joint_velocity_command(4) = 0;

    disp(joint_velocity_command(1))

    robot.writeVelocities(joint_velocity_command)
    pause(robot.dt)
end
pause(robot.dt)
for i = 1:size(velocity_input_2)
    joint_velocity_command = velocity_input_2(i, :)*180/pi;
    % joint_velocity_command(2) = 0;
    % joint_velocity_command(3) = 0;
    % joint_velocity_command(4) = 0;

    disp(joint_velocity_command(1))

    robot.writeVelocities(joint_velocity_command)
    pause(robot.dt)
end
robot.writeVelocities([0,0,0,0])
pause(2)
% current_pos = robot.measured_js(1,0,0)

% jacobian = [
%                         0 0 0 0;
%                         0 1 1 1;
%                         1 0 0 0;
%                         0 0.128 0 0;
%                         0.28 0 0 0;
%                         0 -0.28 -0.25 -0.13;
%                     ]

% FKinBody(robot.M,jacobian,current_pos(1,:).')
% tar_C