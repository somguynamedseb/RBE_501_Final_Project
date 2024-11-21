%% Plotting file
% Input is a matrix of [time, 4joints]
close all
clear

home = [0,0,0,0];
pos = [0, 0,-40,20];
pos2 = [0,-40,40,-20];

RAW_data = readmatrix("lab2_data.csv");

robot = Robot(); % create robot object
live_model = Model(robot); % create live stick model

% XYZData;

for i = 1:length(RAW_data)
    TempTransMatrix = robot.fk3001(RAW_data(i,2:5).'); %make joints -> rot and pos data
    XYZData(i,:) = TempTransMatrix(1:3, 4).'; % extract only XYZ
end

%plot the stick figure in the final position
update_plot(live_model, RAW_data(end, 2:5));

%% plot joints vs time
time = RAW_data(:, 1);
time = time / 1000;

figure;
hold on
for i = 3:5
    jointVals = RAW_data(:, i);     
    plot(time, jointVals);
end
hold off

xlabel("Time (sec)");
ylabel("Joint position (deg)");
title("Joint positition vs time");

legend("Joint 2", "Joint 3", "Joint 4");

%% plot X and Z vs time
figure;
hold on
plot(time, XYZData(:,1));
plot(time, XYZData(:,3));
hold off
xlabel("Time (sec)");
ylabel("Cartesian position (mm)");
title("EE positition vs time");

legend("X pos", "Z pos");

%% XZ scatter plot

% calculate the and extract XYZ theoretical positions using FK3001
% DesiredEE;
TempMatrix = robot.fk3001(home.');
DesiredEE(1,:) = TempMatrix(1:3, 4).';
TempMatrix = robot.fk3001(pos.');
DesiredEE(2,:) = TempMatrix(1:3, 4).';
TempMatrix = robot.fk3001(pos2.');
DesiredEE(3,:) = TempMatrix(1:3, 4).';

figure;
hold on
plot(XYZData(:,1), XYZData(:,3), Color='b');
scatter(DesiredEE(:, 1), DesiredEE(:,3));
hold off

xlabel("X pos (mm)");
ylabel("Z pos (mm)");
title("EE positition in XZ plane");

%% XY scatter plot
figure;
hold on
plot(XYZData(:,1), XYZData(:,2), Color='b');
scatter(DesiredEE(:,1), DesiredEE(:,2));
hold off

axis([180 290 -1 1])

xlabel("X pos (mm)");
ylabel("Y pos (mm)");
title("EE positition in XY plane");















