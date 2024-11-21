clear

% Set up robot
robot = Robot(); % create robot object
robot.writeMotorState(true); % write position mode

%% Gathering Data for .csv
% Positions for joints
home = [0,0,0,0];
pos1 = [0, 0,-40,20];
pos2 = [0,-40,40,-20];

% Move robot to home
robot.interpolate_jp(home,1000);
pause(1);

index = 1;
moveTime = 4500;

% Set up time tracking
tStart = posixtime(datetime('now')) * 1000; % ms
tTrueStart = posixtime(datetime('now')) * 1000; % ms
tNow = posixtime(datetime('now')) * 1000; % ms

% Move robot to first position
robot.interpolate_jp(pos1,4000);
while (tNow - tStart) < moveTime
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);

    % append the position data  
    posData(index,:) = setpoint_js(robot);  %needs to be transposed
    index = index + 1;  
end

% Move robot to second position
robot.interpolate_jp(pos2,4000);
tStart = posixtime(datetime('now')) * 1000; % ms
while (tNow - tStart) < moveTime
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);

    % append the position data  
    posData(index,:) = setpoint_js(robot);  %needs to be transposed
    index = index + 1;   
end

% Move robot to home position
robot.interpolate_jp(home,4000);
tStart =posixtime(datetime('now')) * 1000; % ms
while (tNow - tStart) < moveTime
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);

    % append the position data  
    posData(index,:) = setpoint_js(robot);  %needs to be transposed
    index = index + 1;  
end

% construct the one large matrix
saveMatrix = [time posData];

% write it to a file
writematrix(saveMatrix,'lab2_data.csv','Delimiter',',');
pause(1);






