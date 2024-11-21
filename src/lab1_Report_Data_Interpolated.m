% Set up robot
robot = Robot(); % create robot object
robot.writeMotorState(true); % write position mode

%% Gathering data for .csv
% home and prepare for testing
home = [0, 0, 0, 0];
jPosPlot = [60, -30, -30, 40];
robot.interpolate_jp(home, 1000);
pause(1);

% prepare arrays
time = zeros(1,1);
posData = zeros(1,4); 
moveTime = 2000;
index = 1;
tTrueStart = posixtime(datetime('now')) * 1000;
    
% set target and start moving
robot.interpolate_jp(jPosPlot, moveTime);

% get starting timestamps
tStart = posixtime(datetime('now')) * 1000;
tNow = posixtime(datetime('now')) * 1000;

% collect data for moveTime milliseconds
while (tNow - tStart) < moveTime
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tTrueStart);

    % append the position data
    jointPosNow = measured_js(robot, true, false);
    posData(index, :) = jointPosNow(1, :);

    index = index + 1;    
end

% construct the one large matrix
saveMatrix = [time posData];

% write it to a file
writematrix(saveMatrix,'posDataLabInterpolated4.csv','Delimiter',',');

pause(1);