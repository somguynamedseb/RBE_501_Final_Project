%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RBE_501_Task_4_realtime.m
%
% DESCRIPTION: Moves the robot to a static position, reads the current draw
% of each of the motors, and calculates the approximate torque exerted by 
% each motor. Can also calculate the approximate wrench on the end effector
% based on joint torques.
%
% INPUT: Position, torque constant
%
% OUTPUT: Theoretical joint torques, approximate torque constants
%
% Written by Sebastian Baldini & Brighton Lee & Manoj Velmurugan
% 12/10/2024
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear variables
close all

%% SETUP
% Initialize Robot
robot = Robot();

% Initialize inputs
T_con = 2.1; %% Amps per Nm
thetalist = [0,0,0,0]; % rad
correction_factor_Y = 25; % Fixes the disparity in y-direction readouts



%% MOVE ROBOT
pause(0.5)
robot.interpolate_jp(thetalist, 2000)
pause(3)
vals = robot.measured_js(0,0,1);
pause(1)

%% MAIN LOOP
firstRun = 1;
offset = [];
while 1
    % Read motor currents and convert to amps
    vals = robot.measured_js(0,0,1);
    mA = vals(3,:);
    A = mA/1000;

    % Calculate joint torques
    torque_est = A*T_con;

    % Calculate wrench on end effector
    Ftip = pinv(robot.Blist')*torque_est';

    % Compensates for gravity by subtracting initial reading from future
    % readings
    if firstRun
        offset = Ftip;
        firstRun = 0;
    end
    Ftip = Ftip - offset;

    % Print result (either motor torques or wrench on end effector)
    % fprintf('Motor Torques: %5.4f %5.4f %5.4f %5.4f\n',torque_est(1),torque_est(2),torque_est(3),torque_est(4))
    fprintf('Linear Wrench on End Effector in Base Frame: %5.4f %5.4f %5.4f\n',Ftip(6),-Ftip(5)*correction_factor_Y,-Ftip(4))
    
    pause(0.5)
end