% Set up robot
robot = Robot(); % create robot object
robot.writeMotorState(true); % write position mode

% Program

%% Test servo_jp and goal_js                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
% Test joint values
test_jp1 = [45, 0, 0, 0];
test_jp2 = [-45, -10, -10, -30];

% Go to first set of values, then second set
robot.servo_jp(test_jp1);
goal_js_test_1 = robot.goal_js()
pause(1);

robot.servo_jp(test_jp2);
goal_js_test_2 = robot.goal_js()
pause(1);

%% Test interpolate_jp and setpoint_js
robot.interpolate_jp(test_jp1, 5000);
jointPosDeg = robot.setpoint_js() % get robot joint setpoints in degrees while robot is moving
pause(5);
robot.interpolate_jp(test_jp2, 10000);
pause(10);
 
%% Test measured_js
% Get only position data
jointPos = robot.measured_js(true, false)
 
% Get only velocity data
jointVel = robot.measured_js(false, true)

% Sanity check that 0s are returned if both false
jointNone = robot.measured_js(false, false)

% Get position and velocity data
jointAll = robot.measured_js(true, true)

%% Gathering data for .csv
% home and prepare for testing
home = [0, 0, 0, 0];
jPosPlot = [45, 0, 0, 0];
robot.interpolate_jp(home, 1000);
pause(1);

% prepare arrays
time = zeros(1,1);
posData = zeros(1,4); 
moveTime = 5000; % ms

% set target
robot.interpolate_jp(jPosPlot, moveTime);

% collect move data
tStart = posixtime(datetime('now')) * 1000; % ms
tNow = posixtime(datetime('now')) * 1000; % ms
index = 1;

% collect data for moveTime milliseconds
while (tNow - tStart) < moveTime
    % update and append the current time
    tNow = posixtime(datetime('now')) * 1000;
    time(index, 1) = (tNow - tStart);

    % append the position data
    jointPosNow = measured_js(robot, true, false);
    posData(index, :) = jointPosNow(1, :);
   
    index = index + 1; % move to next row 
end

% construct the one large matrix of time and position
saveMatrix = [time posData];

% write it to a file
writematrix(saveMatrix,'posData.csv','Delimiter',',');

pause(1);
