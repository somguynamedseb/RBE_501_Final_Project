clear; clc;

% Set up robot
robot = Robot();
traj = Traj_Planner();

%% Test end effector positions
% xyza1 = [281.4 0 224.3254 0];
% xyza2 = [273.6898 48.2589 109.1028 30];
% xyza3 = [241.1636 -21.0991 315.0192 -10];
%
% result1 = robot.ik3001(xyza1); % 0 0 0 0
% robot.fk3001(result1);
% robot.interpolate_jp(result1, 1000);
% pause(2);
%
% result2 = robot.ik3001(xyza2); % 10 10 10 10
% robot.fk3001(result2);
% robot.interpolate_jp(result2, 1000);
% pause(2);
%
% result3 = robot.ik3001(xyza3); % -5 -10 -5 -10
% robot.fk3aj.prepareQuinticTraj(traj_runtime, p1joints, p2joints);
traj_23 = tr001(result3);
% robot.interpolate_jp(result3, 1000);
% pause(2);

%% Trajectory planning in Joint Space
xyzp1 = [281.4 0 224.3254 0];
xyzp2 = [250 75 50 20];
xyzp3 = [200 -30 50 20];

p1joints = robot.ik3001(xyzp1);
p2joints = robot.ik3001(xyzp2);
p3joints = robot.ik3001(xyzp3);

traj_runtime = 2000;

%% Move the robot around the triangle with servo_jp (Sign-off 2)

% Move to home position
robot.servo_jp(p1joints);

% Initialize and start timers
tTrueStart = posixtime(datetime('now')) * 1000;
tStart = posixtime(datetime('now')) * 1000;
tNow = posixtime(datetime('now')) * 1000;
index = 1;

while (tNow - tStart) < traj_runtime + 100
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);
    
    % append the position and joint angle data  
    posData(index,:) = robot.setpoint_js().';
    index = index + 1;  
end
pause(1);

robot.servo_jp(p2joints);
tStart = posixtime(datetime('now')) * 1000;
while (tNow - tStart) < traj_runtime + 100
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);
    
    % append the position and joint angle data  
    posData(index,:) = robot.setpoint_js().';
    index = index + 1;  
end
pause(1);

robot.servo_jp(p3joints);
tStart = posixtime(datetime('now')) * 1000;
while (tNow - tStart) < traj_runtime + 100
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);
    
    % append the position and joint angle data  
    posData(index,:) = robot.setpoint_js().';
    index = index + 1;  
end
pause(1);

robot.servo_jp(p1joints);
tStart = posixtime(datetime('now')) * 1000;
while (tNow - tStart) < traj_runtime + 100
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);
    
    % append the position and joint angle data  
    posData(index,:) = robot.setpoint_js().';
    index = index + 1;  
end
pause(1);

% Generating final matrix of data and plotting it
triangleMotionPos = [time posData];
plotLab3(robot, triangleMotionPos);
pause(2);

%% Move the robot around the triangle with run_trajectory(Joint Degrees Input)(Sign-off 3)
% Get trajectory for each side
traj_12 = traj.prepareCubicTraj(traj_runtime, p1joints, p2joints);
traj_23 = traj.prepareCubicTraj(traj_runtime, p2joints, p3joints);
traj_31 = traj.prepareCubicTraj(traj_runtime, p3joints, p1joints);

side1 = robot.run_trajectory(traj_12, traj_runtime, true);
side2 = robot.run_trajectory(traj_23, traj_runtime, true);
side3 = robot.run_trajectory(traj_31, traj_runtime, true);

% Prep data of each side to go together into 1 matrix
side2(:, 1) = side2(:, 1) + side1(end, 1);
side3(:, 1) = side3(:, 1) + side2(end, 1);

whole_triangle = [side1; side2; side3];
plotLab3(robot, whole_triangle);
plotPositionDerivatives(robot, whole_triangle); %SO3

pause(2);
%% Move the robot around the triangle with run_trajectory(Task Space XYZ Input)(Sign-off 4)
traj_12_XYZA = traj.prepareCubicTraj(traj_runtime, xyzp1, xyzp2);
traj_23_XYZA = traj.prepareCubicTraj(traj_runtime, xyzp2, xyzp3);
traj_31_XYZA = traj.prepareCubicTraj(traj_runtime, xyzp3, xyzp1);

side1_XYZA = robot.run_trajectory(traj_12_XYZA, traj_runtime, false);
side2_XYZA = robot.run_trajectory(traj_23_XYZA, traj_runtime, false);
side3_XYZA = robot.run_trajectory(traj_31_XYZA, traj_runtime, false);

% Prep data of each side to go together into 1 matrix
side2_XYZA(:, 1) = side2_XYZA(:, 1) + side1_XYZA(end, 1);
side3_XYZA(:, 1) = side3_XYZA(:, 1) + side2_XYZA(end, 1);

% Generating final matrix of data and plotting it
whole_triangle_XYZA = [side1_XYZA; side2_XYZA; side3_XYZA];
plotLab3(robot, whole_triangle_XYZA);
plotPositionDerivatives(robot, whole_triangle_XYZA); %SO4

% 3D Plot of all the motions
overlayXYZ3Dmotion(robot, triangleMotionPos, whole_triangle, whole_triangle_XYZA);

pause(2);
%% Get trajectory for each side quintic joint (Sign-off 5)
traj_12 = traj.prepareQuinticTraj(traj_runtime, p1joints, p2joints);
traj_23 = traj.prepareQuinticTraj(traj_runtime, p2joints, p3joints);
traj_31 = traj.prepareQuinticTraj(traj_runtime, p3joints, p1joints);

side1 = robot.run_trajectory(traj_12, traj_runtime, true);
side2 = robot.run_trajectory(traj_23, traj_runtime, true);
side3 = robot.run_trajectory(traj_31, traj_runtime, true);

% Prep data of each side to go together into 1 matrix
side2(:, 1) = side2(:, 1) + side1(end, 1);
side3(:, 1) = side3(:, 1) + side2(end, 1);

% Generating final matrix of data and plotting it
whole_triangle = [side1; side2; side3];
plotLab3(robot, whole_triangle);
plotPositionDerivatives(robot, whole_triangle); %SO5

% Quintic with task space
traj_12_XYZA_1 = traj.prepareQuinticTraj(traj_runtime, xyzp1, xyzp2);
traj_23_XYZA_1 = traj.prepareQuinticTraj(traj_runtime, xyzp2, xyzp3);
traj_31_XYZA_1 = traj.prepareQuinticTraj(traj_runtime, xyzp3, xyzp1);

side1_XYZA_1 = robot.run_trajectory(traj_12_XYZA_1, traj_runtime, false);
side2_XYZA_1 = robot.run_trajectory(traj_23_XYZA_1, traj_runtime, false);
side3_XYZA_1 = robot.run_trajectory(traj_31_XYZA_1, traj_runtime, false);

% Prep data of each side to go together into 1 matrix
side2_XYZA_1(:, 1) = side2_XYZA_1(:, 1) + side1_XYZA_1(end, 1);
side3_XYZA_1(:, 1) = side3_XYZA_1(:, 1) + side2_XYZA_1(end, 1);

whole_triangle_XYZA_1 = [side1_XYZA_1; side2_XYZA_1; side3_XYZA_1];
plotLab3(robot, whole_triangle_XYZA_1);
plotPositionDerivatives(robot, whole_triangle_XYZA_1); %SO5

%% EC Circle
circ = robot.runCircle(4000, traj);
plotLab3(robot, circ);
plotPositionDerivatives(robot, circ); %EC

%% Lab 3 Helper Functions
%% Create a Figure with 3 subplots showing the joint space trajectory:
% ● Make a subplot with 3 lines that shows the x, y and z positions (in mm) vs time (in seconds),
% being sure to have unique lines and a legend.
% ● Make a subplot with 3 lines that shows the x, y and z velocities (in mm/s) vs time (in seconds).
% ● Make a subplot with 3 lines that shows the x, y, and z accelerations (in mm/s^2) vs time (in
% seconds).
% Hint: taking the derivative of position with respect to time will get velocity, and taking derivative of
% velocity with respect to time will get acceleration

% takes in raw joint data [time, q1, q2, q3, q4] and plots the x, y, z positions of the end effector
% returns the figure handle
function fig = plotLab3(robot, raw_joint_data)
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

    figure;
    plot(TimeAng,J1,"r-o",TimeAng,J2,"g-*",TimeAng,J3,"k-^",TimeAng,J4,"b-+");
    title("Joint Positions (deg) Over Time");
    legend("Theta 1 (deg)","Theta 2 (deg)","Theta 3 (deg)","Theta 4 (deg)");
    xlabel("Time (ms)");
    ylabel("Angle (degrees)");

    figure;
    plot3(Xe,Ye, Ze);
    title("X, Y, and Z path of End Effector Over Time");
    xlabel("X Position (mm)");
    ylabel("Y Position (mm)");
    zlabel("Z Position (mm)");
    grid on
end

%% Plot position, velocity, and acceleration of x, y, z
function fig = plotPositionDerivatives(robot, raw_joint_data)
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

    fig = figure;

    % Position
    subplot(3, 1, 1);
    plot(Time,Xe,"r",Time,Ye,"g",Time,Ze,"k");
    title("X, Y, Z Positions of End Effector Over Time");
    legend("X pos (mm)","Y pos (mm)","Z pos (mm)");
    xlabel("Time (ms)");
    ylabel("Position (mm)");

    % Get x, y, z velocity
    Xdot = gradient(Xe);
    Ydot = gradient(Ye);
    Zdot = gradient(Ze);

    subplot(3, 1, 2);
    plot(Time, Xdot, "r", Time, Ydot, "g", Time, Zdot, "k");
    title("X, Y, Z Velocities of End Effector Over Time");
    legend("X vel (mm/s)","Y vel (mm/s)","Z vel (mm/s)");
    xlabel("Time (ms)");
    ylabel("Velocity (mm/s)");

    % Get x, y, z acceleration
    Xdot2 = gradient(Xdot);
    Ydot2 = gradient(Ydot);
    Zdot2 = gradient(Zdot);

    subplot(3, 1, 3);
    plot(Time, Xdot2, "r", Time, Ydot2, "g", Time, Zdot2, "k");
    title("X, Y, Z Accelerations of End Effector Over Time");
    legend("X acc (mm/s^2)","Y acc (mm/s^2)","Z acc (mm/s^2)");
    xlabel("Time (ms)");
    ylabel("Acceleration (mm/s^2)");
end

function overlayXYZ3Dmotion(robot, raw_motion1, raw_motion2, raw_motion3)
    % get the XYZ data from the 3 motions
    figure;
    hold on

    for mtn = 1:3  
        %select the motion data, and reset the matrixXYZ
        if(mtn == 1)
            saveMatrixAng = raw_motion1;
            saveMatrixPos = zeros(size(saveMatrixAng, 1),4);
            color = 'r';
        elseif(mtn == 2)
            saveMatrixAng = raw_motion2; 
            saveMatrixPos = zeros(size(saveMatrixAng, 1),4);
            color = 'g';
        else
            saveMatrixAng = raw_motion3;
            saveMatrixPos = zeros(size(saveMatrixAng, 1),4);
            color = 'b';
        end
    
        % calculate the end effector positions
        for i = 1:size(saveMatrixAng, 1)
            tempVar = robot.fk3001(saveMatrixAng(i,2:5));
            saveMatrixPos(i,1) = (saveMatrixAng(i,1));   % save the time
            saveMatrixPos(i,2:4) = tempVar(1:3, 4);   % save the x, y, z positions
        end

        % prepare the separate XYZ data
        Xe = saveMatrixPos(:,2);
        Ye = saveMatrixPos(:,3);
        Ze = saveMatrixPos(:,4);
        
        plot3(Xe,Ye, Ze, "Color", color);
    end

    title("X, Y, and Z path of End Effector Over Time for 3 Motions");
    xlabel("X Position (mm)");
    ylabel("Y Position (mm)");
    zlabel("Z Position (mm)");
    legend("Fast Motion", "Cubic Motion Joint", "Cubic Motion Task");
    grid on
    hold off
    view(3);
end
