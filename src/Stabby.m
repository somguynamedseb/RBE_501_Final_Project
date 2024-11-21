close all;
clear;
clc;

% Set up robot and go home
robot = Robot();
cal_home = [-90, -30, 85, -45];
robot.writeGripper(1);
pause(0.5);
robot.interpolate_jp(cal_home, 750);
pause(0.5);
robot.writeGripper(0);

% Set up camera
cam = Camera();

% Put down the knife and the duck for the robot
disp("Time to move s--t");
pause;

% Find the knife
KnifeLocation = cam.stab(0);
robot.writeGripper(1);
pose = cam.checker_to_robot(cam.cam_to_checker(KnifeLocation(1,:))); 
currentTargetXYZA = cam.remove_xy_offset(pose);

% Grab the knife the knife
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 90, 90)),1000);
pause(1.5);
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA,20,90)),1000);
pause(1.5);
robot.writeGripper(0);
pause(1.5);

% Pick the knife up slowly, then menacingly
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA,90,90)),1000);
pause(1.5);
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA,300,0)),300);

disp("Awaiting orders");
pause;

% Find the duck and hover over it
duckSpot = cam.stab(1);
pose = cam.checker_to_robot(cam.cam_to_checker(duckSpot(1,:))); 
currentTargetXYZA = cam.remove_xy_offset(pose);
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 300, 0)),1000);
pause(1.2);

% Stab
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 150, 0)),1000);
pause(1.2);
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 200, 0)),1000);
pause(1.2);
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 150, 0)),1000);
pause(1.2);
robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 200, 0)),1000);
pause(1.2);
