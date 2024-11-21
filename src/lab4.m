clear; clc;

%% Code for lab 4 testing
robot = Robot();
robot_model = Model(robot);
traj = Traj_Planner();
pause(1); % pause for setup

%% Sign off 1
% Joint arrays for overhead position and second test case
q_overhead = [0 0 -90 0];
q_second = [0 0 0 0];

% Calculate Jacobian
test_overhead = robot.jakub3001(q_overhead)
test_second = robot.jakub3001(q_second)

colrow3_overhead = test_overhead(1:3, 1:3);
colrow3_test = test_second(1:3, 1:3);

% Determinant of first three columns of Jp (upper half)
overhead3x3 = det(colrow3_overhead)
second3x3 = det(colrow3_test)

%% SO2 stuff
xyzp1 = [281.4 0 224.3254 0];
xyzp2 = [250 75 50 20];
xyzp3 = [200 -30 50 20];
traj_runtime = 6000;

% Set robot to home position
robot.interpolate_jp(robot.ik3001(xyzp1), 1000);
pause(1);

% Prepare quintic trajectories for triangle movement
traj_12_XYZA_1 = traj.prepareQuinticTraj(traj_runtime, xyzp1, xyzp2);
traj_23_XYZA_1 = traj.prepareQuinticTraj(traj_runtime, xyzp2, xyzp3);
traj_31_XYZA_1 = traj.prepareQuinticTraj(traj_runtime, xyzp3, xyzp1);

% Run robot through the triangle
side1_XYZA_1 = robot.run_trajectory(traj_12_XYZA_1, traj_runtime, false, robot_model);
side2_XYZA_1 = robot.run_trajectory(traj_23_XYZA_1, traj_runtime, false, robot_model);
side3_XYZA_1 = robot.run_trajectory(traj_31_XYZA_1, traj_runtime, false, robot_model);

% Graphs for sign-off 2
plotLinAngScalarVel(side1_XYZA_1, side2_XYZA_1, side3_XYZA_1);

%% S03 stuff
% Get home and singularity positions and send robot home
p1joints = [0 0 0 0];
p2joints = [0 0 -90 0];
robot.interpolate_jp(p1joints, 800);
pause(2);

% Prepare trajectory from home to singularity
traj_error_test = traj.prepareQuinticTraj(6000, p1joints, p2joints);
pause(1);

% Run robot to singularity with e-stop and save the data
saveMatrix = robot.run_trajectory_Estop(traj_error_test,6000,true, robot_model);
pause(3);

% Send robot home
interpolate_jp(robot,p1joints,2000);
pause(2);

% Plots for SO3
plotLab4SO3(robot,saveMatrix(:,1:5));
figure;

plot(saveMatrix(:, 1), saveMatrix(:, 6));
title("Determinat of 3x3 Jakubian Submatrix");
xlabel("Time (ms)");
ylabel("Determinant value");
    
%% Helper Functions
function fig = plotLinAngScalarVel(side1, side2, side3)
    % Fix timestamps for combination and then combine
    side2(:, 1) = side2(:, 1) + side1(end, 1);
    side3(:, 1) = side3(:, 1) + side2(end, 1);
    whole_triangle = [side1; side2; side3];

    % Get all velocity variables for all graphs
    time = whole_triangle(:, 1);
    xdot = whole_triangle(:, 2);
    ydot = whole_triangle(:, 3);
    zdot = whole_triangle(:, 4);
    wx = whole_triangle(:, 5);
    wy = whole_triangle(:, 6);
    wz = whole_triangle(:, 7);

    % Get scalar speed of the end effector
    scalar_speed = sqrt(xdot.^2 + ydot.^2 + zdot.^2);

    fig = figure;

    % Linear velocity
    subplot(3, 1, 1);
    plot(time, xdot, "r", time, ydot, "g", time, zdot, "k");
    title("Linear X, Y, and Z Velocity of End Effector Over Time");
    legend("X vel (mm/s)","Y vel (mm/s)","Z vel (mm/s)");
    xlabel("Time (ms)");
    ylabel("Velocity (mm/s)");


    % Angular velocity
    subplot(3, 1, 2);
    plot(time, wx, "r", time, wy, "g", time, wz, "k");
    title("Angular X, Y, and Z Velocity of End Effector Over Time");
    legend("X vel (rad/s)","Y vel (rad/s)","Z vel (rad/s)");
    xlabel("Time (ms)");
    ylabel("Velocity (rad/s)");

    % Scalar speed
    subplot(3, 1, 3);
    plot(time, scalar_speed);
    title("Scalar Speed of the End Effector Over Time");
    legend("Scalar Speed (mm/s)");
    xlabel("Time (ms)");
    ylabel("Velocity (mm/s)");
end

function fig = plotLab4SO3(robot, raw_joint_data)
    saveMatrixAng = raw_joint_data;
    saveMatrixPos = zeros(size(saveMatrixAng,1),4);

    % calculate the end effector positions
    for i = 1:size(saveMatrixAng, 1)
        tempVar = robot.fk3001(saveMatrixAng(i,2:5));
        saveMatrixPos(i,1) = (saveMatrixAng(i,1));   % save the time
        saveMatrixPos(i,2:4) = tempVar(1:3, 4);   % save the x, y, z positions
    end

    Time = saveMatrixPos(:,1);
    Xe = saveMatrixPos(:,2);
    Ye = saveMatrixPos(:,3);
    Ze = saveMatrixPos(:,4);
    
    % Plot x, y, z of end effector over time (individual)
    fig = figure;
    plot(Time,Xe,"r",Time,Ye,"g",Time,Ze,"k");
    title("X, Y, Z Positions of End Effector Over Time");
    legend("X pos (mm)","Y pos (mm)","Z pos (mm)");
    xlabel("Time (ms)");
    ylabel("Position (mm)");

    % Get joint angle data
    TimeAng = saveMatrixAng(:,1);
    J1 = saveMatrixAng(:,2);
    J2 = saveMatrixAng(:,3);
    J3 = saveMatrixAng(:,4);
    J4 = saveMatrixAng(:,4);

    % Plot joint position vs time
    figure;
    plot(TimeAng,J1,"r-o",TimeAng,J2,"g-*",TimeAng,J3,"k-^",TimeAng,J4,"b-+");
    title("Joint Positions (deg) Over Time");
    legend("Theta 1 (deg)","Theta 2 (deg)","Theta 3 (deg)","Theta 4 (deg)");
    xlabel("Time (ms)");
    ylabel("Angle (degrees)");
    
    % Plot x, y, z of EE over time (3D)
    figure;
    plot3(Xe,Ye, Ze);
    title("X, Y, and Z path of End Effector Over Time");
    xlabel("X Position (mm)");
    ylabel("Y Position (mm)");
    zlabel("Z Position (mm)");
    grid on;
    ylim([-20   20]);
end