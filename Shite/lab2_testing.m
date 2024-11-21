% Set up robot
robot = Robot(); % create robot object
robot.writeMotorState(true); % write position mode
home = [0,0,0,0];
pos1 = [30,0,-30,30];
pos2 = [0,-30,0,30];
robot.interpolate_jp(home,1000);
pause(1);

%% Testing measured_cp
robot.interpolate_jp(pos1,1000);
pause(2);
test1 = measured_js(robot,true,false)
Matrix_measured_1 = measured_cp(robot)
robot.interpolate_jp(pos2,1000);
pause(2);
Matrix_measured_2 = measured_cp(robot)
robot.interpolate_jp(home,1000);
pause(2);

%% Testing setpoint_cp
robot.interpolate_jp(pos1,1000);
pause(2);
test2 = setpoint_js(robot)
Matrix_setpoint_1 = setpoint_cp(robot)
robot.interpolate_jp(pos1,1000);
pause(2);
Matrix_setpoint_2 = setpoint_cp(robot)
robot.interpolate_jp(home,1000);
pause(2);

%% Testing goal_cp
robot.interpolate_jp(pos1,1000);
pause(2);
Matrix_goal_1 = goal_cp(robot)
robot.interpolate_jp(pos1,1000);
pause(2);
Matrix_goal_2 = goal_cp(robot)
robot.interpolate_jp(home,1000);
pause(2);