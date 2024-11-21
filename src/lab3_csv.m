clear

% Set up robot
robot = Robot(); % create robot object
robot.writeMotorState(true); % write position mode

%% Live model setup
robot_model = Model(robot);

%% Gathering Data for .csv
%  Position for EE and alpha_EE Values
xyzp1 = [281.4 0 224.3254 0];
xyzp2 = [250 75 50 20];
xyzp3 = [200 -30 50 20];

% Move robot to home
result1 = robot.ik3001(xyzp1);
robot.fk3001(result1)
robot.interpolate_jp(result1, 1000);
robot_model.update_plot(setpoint_js(robot));
pause(1);

index = 1;
moveTime = 3000;

% Set up time tracking
tStart = posixtime(datetime('now')) * 1000; % ms
tTrueStart = posixtime(datetime('now')) * 1000; % ms
tNow = posixtime(datetime('now')) * 1000; % ms

% Move robot to first position
result2 = robot.ik3001(xyzp2); 
robot.interpolate_jp(result2, moveTime);
while (tNow - tStart) < moveTime + 100
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);
    % update robot live 3d plot
    robot_model.update_plot(setpoint_js(robot));
    pause(0.01);
    
    % append the position and joint angle data  
    angData(index,:) = setpoint_js(robot).';
    tempPosData = setpoint_cp(robot);
    posData(index,:) = tempPosData(1:3,4).';  %needs to be transposed
    index = index + 1;  
end

% Move robot to second position
result3 = robot.ik3001(xyzp3);
robot.fk3001(result3)
robot.interpolate_jp(result3, moveTime);
tStart = posixtime(datetime('now')) * 1000; % ms
while (tNow - tStart) < moveTime + 100
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);
    % update robot live 3d plot
    robot_model.update_plot(setpoint_js(robot));
    pause(0.01);
    
    % append the position and joint angle data  
    angData(index,:) = setpoint_js(robot);
    tempPosData = setpoint_cp(robot);
    posData(index,:) = tempPosData(1:3,4).';  %needs to be transposed
    index = index + 1; 
end

% Move robot to home position
result1 = robot.ik3001(xyzp1);
robot.interpolate_jp(result1, moveTime);
tStart =posixtime(datetime('now')) * 1000; % ms
while (tNow - tStart) < moveTime + 100
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);
    % update robot live 3d plot
    robot_model.update_plot(setpoint_js(robot));
    pause(0.01);
    
    % append the position and joint angle data  
    angData(index,:) = setpoint_js(robot).';
    tempPosData = setpoint_cp(robot);
    posData(index,:) = tempPosData(1:3,4).';  %needs to be transposed
    index = index + 1;  
end

% construct the one large matrix
saveMatrixPos = [time posData];
saveMatrixAng = [time angData];

% write it to a file
writematrix(saveMatrixPos,'lab3_data_pos.csv','Delimiter',',');
pause(1);

% Get end effector position data
Time = saveMatrixPos(:,1);
Xe = saveMatrixPos(:,2);
Ye = saveMatrixPos(:,3);
Ze = saveMatrixPos(:,4);

figure
plot(Time,Xe,"r",Time,Ye,"g",Time,Ze,"k")
title("X, Y, Z Positions of End Effector Over Time");
legend("X pos (mm)","Y pos (mm)","Z pos (mm)")
xlabel("Time (ms)");
ylabel("Position (mm)");

% Get joint angle data
TimeAng = saveMatrixAng(:,1);
J1 = saveMatrixAng(:,2);
J2 = saveMatrixAng(:,3);
J3 = saveMatrixAng(:,4);
J4 = saveMatrixAng(:,4);

figure
plot(TimeAng,J1,"r-o",TimeAng,J2,"g-*",TimeAng,J3,"k-^",TimeAng,J4,"b-+")
title("Joint Positions (deg) Over Time");
legend("Theta 1 (deg)","Theta 2 (deg)","Theta 3 (deg)","Theta 4 (deg)")
xlabel("Time (ms)");
ylabel("Angle (degrees)");

figure
plot3(Xe,Ye, Ze)
title("X, Y, and Z path of End Effector Over Time");
xlabel("X Position (mm)");
ylabel("Y Position (mm)");
zlabel("Z Position (mm)");
grid on


