%% Set up robot object
robot = Robot();

%% Setup Motion and Data Collection
home = [0; 0; 0; 0];
pos = [-45;-30;30;30];
robot.interpolate_jp(home,1000)
pause(1);
theoVals = robot.fk3001(home);
theoVals = theoVals(1:3,4);
for i = 1:5
    robot.interpolate_jp(pos,1000)
    pause(1);
    robot.interpolate_jp(home,1000)
    pause(1.1);
    temp = measured_cp(robot);
    posData(i,:)  = temp(1:3,4).'
    saveMatrix = posData;
end    
writematrix(saveMatrix,'lab2_data_repeated.csv','Delimiter',',');
pause(0.5)

%% Final Calculations
PosX = saveMatrix(:,1);
PoxY = saveMatrix(:,2);
PozZ= saveMatrix(:,3);
temp = 0;
RMStemp = 0;

for i = 1:5
    temp = PosX(i) +temp;
    RMStemp = (PosX(i))^2 +RMStemp;
end
avgPosX = temp/5
PosX_RMS = sqrt(RMStemp/5)-theoVals(1)
temp = 0;
RMStemp = 0;
for i = 1:5
    temp = PoxY(i) +temp;
    RMStemp = (PoxY(i))^2 +RMStemp;
end
PosY_RMS = sqrt(RMStemp/5) -theoVals(2)
avgPosY = temp/5
temp = 0;
RMStemp = 0;
for i = 1:5
    temp = PozZ(i) +temp;
    RMStemp = (PozZ(i))^2 +RMStemp;
end
avgPosZ = temp/5
PosZ_RMS = sqrt(RMStemp/5) -theoVals(3)

%% 3d Plot Generation
scatter3(PosX(:), PoxY(:), PozZ(:))
ylim([-1 1])
ylabel("YPos(mm)")
xlim([270 290])
xlabel("XPos(mm)")
zlim([210 220])
zlabel("ZPos(mm)")
title("Home Position Repeatability ;)")
