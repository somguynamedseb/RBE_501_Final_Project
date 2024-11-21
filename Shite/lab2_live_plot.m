%% Set up robot object
robot = Robot();

%% Live model setup
robot_model = Model(robot);

% Positions for testing that plot_arm() works
q_home = [0; 0; 0; 0];
pos = [-45; -90; 90; -90];
pos2 = [45; -45; 45; 60];

% Positions for testing that plot_arm() works with the physical arm
testPos1 =[0;-10;-10;30];
testPos2 = [10;-10;-20;30];
testPos3 = [20;-20;-30;40];
testPos4 =[30;-30;0;45];
testPos5 = [40; -20; 20; 55];

% Move the arm
robot.interpolate_jp(q_home,1000);
pause(2);

% Move the stick model
liveRobotMove(testPos2, 10000, robot, robot_model);

function liveRobotMove(q, moveTime, robot, robot_model)
    q

    % set target
    robot.interpolate_jp(q, moveTime);
    
    % collect move data
    tStart = posixtime(datetime('now')) * 1000; % ms
    tNow = posixtime(datetime('now')) * 1000; % ms
    index = 1;

    % collect data for moveTime milliseconds
    while (tNow - tStart) < moveTime
        % update and append the current time
        tNow = posixtime(datetime('now')) * 1000;

        % update the stick model
        robot_model.update_plot(setpoint_js(robot));
        pause(0.067);

        index = index + 1; % move to next row 
    end

    % write out the setpoints and FK
    pause(1);
    current_setpoint = robot.setpoint_js()
    end_pos_fk = robot.fk3001(robot.setpoint_js())
    pause(1);
end





