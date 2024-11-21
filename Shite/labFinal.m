close all;
clear;
clc;

robot = Robot();

% move the robot to home position
jHome = [-90, -30, 85, -45];
robot.interpolate_jp(jHome, 750);
pause(0.5);

% Set up camera
cam = Camera();
traj = Traj_Planner();

% sorting position definitions in XYZA
% defined in the color order: blue, yellow, red, green, gray, orange
posSortDropoff = [50, -225, 120, 0;
                100, -225, 120, 0;
                150, -225, 120, 0;
                200, -225, 120, 0;
                250, -225, 120, 0;
                300, -225, 120, 0];

% routine to repeat: find ball, open, go above, down, close, up, go to dropoff, home
pause(1);
disp("Add ballz and press the power button to continue");
pause;

while(true)
    % get the ball positions
    bCoords = [];
    yCoords = [];
    rCoords = [];
    gCoords = [];
    grCoords = [];
    oCoords = [];
    [bCoords, yCoords, rCoords, gCoords, grCoords, oCoords] = cam.getBalls(1);   % blue, yellow, red, green, gray, orange    

    %check if there are any blue balls
    if(size(bCoords,1) > 0)
        sort1ball(cam, robot, bCoords(1,:), posSortDropoff(1,:));
    
    %check if there are any yellow balls
    elseif(size(yCoords,1) > 0)
        sort1ball(cam, robot, yCoords(1,:), posSortDropoff(2,:));

    %check if there are any red balls
    elseif(size(rCoords,1) > 0)
        sort1ball(cam, robot, rCoords(1,:), posSortDropoff(3,:));

    %check if there are any green balls
    elseif(size(gCoords,1) > 0)
        sort1ball(cam, robot, gCoords(1,:), posSortDropoff(4,:));

    %check if there are any gray balls
    elseif(size(grCoords,1) > 0)
        sort1ball(cam, robot, grCoords(1,:), posSortDropoff(5,:));

    %check if there are any orange balls
    elseif(size(oCoords,1) > 0)
        sort1ball(cam, robot, oCoords(1,:), posSortDropoff(6,:));
    else
        % we are done sorting, exit the loop        
        break;
    end
end

%-----slow end park mode

pause(10);
robot.writeGripper(0); % close
disp("Creeping...");
robot.interpolate_jp(jHome, 8000);

%--------------------------------

function sort1ball(cam, robot, ballCheckerXY, dropoffXYZA)
    moveTime = 750;
    clawTime = 500;
    jHome = [-90, -30, 85, -45];

    % get the ball position in the robot frame
    offsetTargetXYZA = cam.checker_to_robot(cam.cam_to_checker(ballCheckerXY));
    % correct for camera lens rays
    currentTargetXYZA = cam.remove_xy_offset(offsetTargetXYZA);

    % move above
    robot.writeGripper(1); % open
    robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 50, 90)), moveTime);
    pause(moveTime/800);
    
    % collect ball
    robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 15, 90)), moveTime);
    pause(moveTime/800);
    robot.writeGripper(0); % close
    pause(clawTime/1000);

    % lift ball up
    robot.interpolate_jp(robot.ik3001(robot.appro_za(currentTargetXYZA, 80, 90)), moveTime/2);
    pause(moveTime/800/2);

    % drop ball off at designated spot
    robot.interpolate_jp(robot.ik3001(dropoffXYZA), moveTime);
    pause(moveTime/800);
    robot.writeGripper(clawTime/1000); % open

    % move out of the way
    robot.interpolate_jp(jHome, moveTime);
    pause(clawTime/1000);
end
