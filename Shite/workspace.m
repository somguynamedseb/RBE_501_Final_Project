clear

% Set up robot
robot = Robot(); % create robot object
robot.writeMotorState(true); % write position mode

%% Gathering Data for .csv
% Positions for joints
home = [20,0,0,120];
posmax = [10, -90,90,00];
posmin = [10, 90,-100,-100];

% % Move robot to home
% robot.interpolate_jp(home,1000);
% pause(1);
% robot.interpolate_jp(posmax,1000);
% pause(1);
% robot.interpolate_jp(posmin,1000);
% pause(1);

%% tested maximum joint values
n_points = 20;
joint1 = linspace(-90, 90, n_points);
joint2 = linspace(-90, 90, n_points);
joint3 = linspace(-100, 90, n_points);
joint4 = linspace(-100, 120, n_points);

GigaArray = zeros(n_points ^ 4, 3);
index = 1;

% generate all the possible positions of the robot
for j1 = joint1
    for j2 = joint2
        for j3 = joint3
            for j4 = joint4
                Temp = robot.fk3001([j1, j2, j3, j4]);
                GigaArray(index,:) = Temp(1:3,4);
                index = index + 1;
            end
        end
    end
end

% find a boundary with shrink factor 1
% of all the possible reachable points
% note: takes about 2mins to graph
% figure("Name","Workspace");    
k = boundary(GigaArray,1);
fig_ref = trisurf(k,GigaArray(:,1),GigaArray(:,2),GigaArray(:,3), ...
'Facecolor','red', 'FaceAlpha',0.15,'LineWidth', 0.05);

view([(-37.5 + 160) 25])
axis equal;
xlabel("X Axis (mm)");
ylabel("Y Axis (mm)");
zlabel("Z Axis (mm)");
title("HALL 4000 Workspace");


