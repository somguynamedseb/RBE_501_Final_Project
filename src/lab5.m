close all;
clear;
clc;

% Set up robot and go home
robot = Robot();
cal_home = [-90, -30, 85, -45];
robot.interpolate_jp(cal_home, 750);
pause(0.5);

% Set up camera and get grid points
cam = Camera();
grid_points = cam.getGridPoints();

try
    %% Sign off 2
    % Get 4 test points in image x, y before fisheye
    p1 = [grid_points(1,1),grid_points(1,2)];
    p2 = [grid_points(4,1),grid_points(4,2)];
    p3 = [grid_points(37,1),grid_points(37,2)];
    p4 = [grid_points(40,1),grid_points(40,2)];
    
    % Get world points for each point
    p1r = cam.checker_to_robot(cam.cam_to_checker(p1));
    p1r = robot.appro_za(p1r, 30, 90);
    
    p2r = cam.checker_to_robot(cam.cam_to_checker(p2));
    p2r = robot.appro_za(p2r, 30, 90);
    
    p3r = cam.checker_to_robot(cam.cam_to_checker(p3));
    p3r = robot.appro_za(p3r, 30, 90);
    
    p4r = cam.checker_to_robot(cam.cam_to_checker(p4));
    p4r = robot.appro_za(p4r, 30, 90);

    % Move the robot to those positions
    timeToMove = 700;

    robot.interpolate_jp(robot.ik3001(p1r), timeToMove);
    pause(timeToMove / 800);
    robot.interpolate_jp(robot.ik3001(p2r), timeToMove);
    pause(timeToMove / 800);
    robot.interpolate_jp(robot.ik3001(p3r), timeToMove);
    pause(timeToMove / 800);
    robot.interpolate_jp(robot.ik3001(p4r), timeToMove);
    pause(timeToMove / 800);
    robot.interpolate_jp(robot.ik3001(p1r), timeToMove);
    pause(timeToMove / 800);   
 
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

%% Sign-off 3
disp("Add ballz and press the power button to continue");
pause;
[bCoords, yCoords, rCoords, gCoords, grCoords, oCoords] = cam.getBalls(1);   % blue, yellow, red, green, gray, orange

robot.writeGripper(1);

%% Sign-off 4
pose = cam.checker_to_robot(cam.cam_to_checker(gCoords(1,:))); % using green ball as the test ball
r = 11.5; % ball radius in mm
h = 175; % camera lens height in mm
cam_x = 340; % camera distance from robot in x direction

if(pose(2) < 0)
    adjust = (r/h);
else
    adjust = -(r/h);
end

adjusted_pose = [pose(1)+(r/h)/3*(cam_x-pose(1)) pose(2)+adjust*pose(2)-3 pose(3) pose(4)]; % pose with adjusted x, y

% go over the point
robot.interpolate_jp(robot.ik3001(robot.appro_za(adjusted_pose, 50, 90)), 900);
pause(1.5);

% Yoink the ball
robot.interpolate_jp(robot.ik3001(robot.appro_za(adjusted_pose, 14, 90)), 800);
pause(1);
robot.writeGripper(0);

% Go home and drop the ball
pause(2);
disp("Creeping...")
robot.interpolate_jp(cal_home, 4000);
pause(2);
robot.writeGripper(1);
