close all;
clear;
clc;

% Set up robot and go home
robot = Robot();
cal_home = [-90, -30, 85, -45];
robot.interpolate_jp(cal_home, 750);
pause(0.5);

% Set up camera
cam = Camera();

disp("Place the ball in the field, hit enter, and start moving the ball.");
pause;
gCoords = cam.getGreenBalls(); % tracking only green balls for simplicity

% Endlessly track the green ball
while (true)
    if (isempty(gCoords) == 0) % if there is a green ball detected
        % Get its position and move to being above it
        pose = cam.checker_to_robot(cam.cam_to_checker(gCoords(1,:))); 
        currentTargetXYZA = cam.remove_xy_offset(pose);
        robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 90, 90)), 90);
    else
       % pause(.05);
    end
    
    gCoords = cam.getGreenBalls(); % check again for the ball
end



