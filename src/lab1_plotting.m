close all
clear all

% Load data
dataInterp1 = readmatrix("posDataLabInterpolated1.csv");
dataInterp2 = readmatrix("posDataLabInterpolated2.csv");
dataInterp3 = readmatrix("posDataLabInterpolated3.csv");
dataInterp4 = readmatrix("posDataLabInterpolated4.csv");

dataNonInterp1 = readmatrix("posDataLabNonInterpolated1.csv");
dataNonInterp2 = readmatrix("posDataLabNonInterpolated2.csv");
dataNonInterp3 = readmatrix("posDataLabNonInterpolated3.csv");
dataNonInterp4 = readmatrix("posDataLabNonInterpolated4.csv");

%% Historam for each run, all in one gaint plot
fig = figure;

index = 1;

% Use while loop and switch case to go through all 8 tests
while index < 9
    subplot(4, 2, index);
    
    switch index
        case 1
            currMartix = dataInterp1;
        case 2
            currMartix = dataInterp2;
        case 3
            currMartix = dataInterp3;
        case 4
            currMartix = dataInterp4;   
        case 5
            currMartix = dataNonInterp1;
        case 6
            currMartix = dataNonInterp2;
        case 7
            currMartix = dataNonInterp3;
        case 8
            currMartix = dataNonInterp4;
    end
    
    % Get time data
    xAxis = currMartix(:,1);
    clear timesteps;
    
    % calculate timesteps
    for i = 1:1:(length(xAxis) - 1)
        timesteps(i) = abs(xAxis(i) - xAxis(i+1));
    end
    
    % Set up histogram
    nBinSize = [47, 47:0.1:49 49];
    histogram(timesteps, nBinSize)
    
    xlabel("Update Time (ms)");
    ylabel("No. of Updates");  
    
    ylim([0 20]);
    
    % Get min, max, mean, median and add to graph
    maximum = max(timesteps);       
    minimum = min(timesteps);
    meain_val = mean(timesteps);
    median_val = median(timesteps);

    txt = {"Mean: " + meain_val, "Median: " + median_val, "Max: " + maximum, "Min: " + minimum };
    text(47,15,txt)

    % Organize and title graphs based on movement type
    if index < 5  
        title("Interpolated Movement " + index + " Update Timestep")
    else
        title("Non-interpolated Movement " + (index - 4) + " Update Timestep")
    end

    index = index + 1;
end

% Label the subplots
final = axes(fig, 'visible', 'off');
final.Title.Visible='on';

title(final,"Histograms of Update Timestep (ms)");

%% Base Joint Interpolated Motion
% Load data
xInterp = dataInterp1(:, 1);
yInterp1 = dataInterp1(:, 2);
yInterp2 = dataInterp2(:, 2);
yInterp3 = dataInterp3(:, 2);
yInterp4 = dataInterp4(:, 2);

% Start figure
figure
plot(xInterp,yInterp1);
hold on;
plot(xInterp,yInterp2);
plot(xInterp,yInterp3);
plot(xInterp,yInterp4);
hold off;

% Label figure
ylabel("Joint Position (Degrees)")
xlabel("Time (ms)")
title("Joint Position (Degrees) vs. Time(ms) of Interpolated Motion")
xlim([0 2100]);
legend({'Run 1','Run 2','Run 3','Run 4'},'Location','southeast')

%% Base Joint NonInterpolated Motion
% Load data
xNonInterp = dataNonInterp1(:, 1);
yNonInterp1 = dataNonInterp1(:, 2);
yNonInterp2 = dataNonInterp2(:, 2);
yNonInterp3 = dataNonInterp3(:, 2);
yNonInterp4 = dataNonInterp4(:, 2);

% Start figure
figure
plot(xNonInterp,yNonInterp1);
hold on;
plot(xNonInterp,yNonInterp2);
plot(xNonInterp,yNonInterp3);
plot(xNonInterp,yNonInterp4);
hold off;

% Label figure
ylabel("Joint Position (Degrees)")
xlabel("Time (ms)")
title("Joint Position (Degrees) vs. Time(ms) of NonInterpolated Motion")
xlim([0 2100]);
legend({'Run 1','Run 2','Run 3','Run 4'},'Location','southeast')

%% All Joints Motion Graphed Interpolated
% All 4 joints, first run
figure
for i = 2:5
    % Load joint and time data
    xInterp = dataInterp1(:, 1);
    yInterp = dataInterp1(:, i);
    
    % Add joint i data to graph
    subplot(4,1,i-1)
    plot(xInterp, yInterp);
    
    % Add labels
    ylabel("Joint Position")
    xlabel("Time (ms)")
    title("Joint Position (Degrees) vs. Time(ms) Joint" + (i-1) + " Run 1")
end

% All 4 joints, second run
figure
for i = 2:5
    xInterp = dataInterp2(:, 1);
    yInterp = dataInterp2(:, i);
    subplot(4,1,i-1)
    plot(xInterp, yInterp);
    ylabel("Joint Position")
    xlabel("Time (ms)")
    title("Joint Position (Degrees) vs. Time(ms) Joint" + (i-1)+ " Run 2")
end

% All 4 joints, third run
figure
for i = 2:5
    xInterp = dataInterp3(:, 1);
    yInterp = dataInterp3(:, i);
    subplot(4,1,i-1)
    plot(xInterp, yInterp);
    ylabel("Joint Position")
    xlabel("Time (ms)")
    title("Joint Position (Degrees) vs. Time(ms) Joint" + (i-1)+ " Run 3")
end

% All 4 joints, fourth run
figure
for i = 2:5
    xInterp = dataInterp4(:, 1);
    yInterp = dataInterp4(:, i);
    subplot(4,1,i-1)
    plot(xInterp, yInterp);
    ylabel("Joint Position")
    xlabel("Time (ms)")
    title("Joint Position (Degrees) vs. Time(ms) Joint" + (i-1)+ " Run 4")
end

%% All Joints Motion Graphed NonInterpolated

% All 4 joints, first run
figure
for i = 2:5
    xNonInterp = dataNonInterp1(:, 1);
    yNonInterp = dataNonInterp1(:, i);
    subplot(4,1,i-1)
    plot(xNonInterp, yNonInterp);
    ylabel("Joint Position")
    xlabel("Time (ms)")
    title("Joint Position (Degrees) vs. Time(ms) Joint" + (i-1)+ " Run 1")
end

% All 4 joints, second run
figure
for i = 2:5
    xNonInterp = dataNonInterp2(:, 1);
    yNonInterp = dataNonInterp2(:, i);
    subplot(4,1,i-1)
    plot(xNonInterp, yNonInterp);
    ylabel("Joint Position")
    xlabel("Time (ms)")
    title("Joint Position (Degrees) vs. Time(ms) Joint" + (i-1)+ " Run 2")
end

% All 4 joints, third run
figure
for i = 2:5
    xNonInterp = dataNonInterp3(:, 1);
    yNonInterp = dataNonInterp3(:, i);
    subplot(4,1,i-1)
    plot(xNonInterp, yNonInterp);
    ylabel("Joint Position")
    xlabel("Time (ms)")
    title("Joint Position (Degrees) vs. Time(ms) Joint" + (i-1)+ " Run 3")
end

% All 4 joints, fourth run
figure
for i = 2:5
    xNonInterp = dataNonInterp4(:, 1);
    yNonInterp = dataNonInterp4(:, i);
    subplot(4,1,i-1)
    plot(xNonInterp, yNonInterp);
    ylabel("Joint Position")
    xlabel("Time (ms)")
    title("Joint Position (Degrees) vs. Time(ms) Joint" + (i-1)+ " Run 4")
end

%% Position of all four joints for all four interpolated and non-interpolated movements
figure;
hold on

% Interpolated run 1
for i = 2:5
    % Load joint and time data
    xInterp = dataInterp1(:, 1);
    yInterp = dataInterp1(:, i);
    
    % Add joint i data to graph
    plot(xInterp, yInterp);
end

% Non-Interpolated run 1
for i = 2:5
    % Load joint and time data
    xInterp = dataNonInterp1(:, 1);
    yInterp = dataNonInterp1(:, i);
    
    % Add joint i data to graph
    plot(xInterp, yInterp);
end
hold off;

% Add labels
xlim([0 2100]);
xlabel("Time (ms)");
ylabel("Joint Position (Degrees)");
title("Joint Position vs. Time for Interpolated and Non-Interpolated Movement (Run 1)");
legend({"J1-I", "J2-I", "J3-I", "J4-I", "J1-NI", "J2-NI", "J3-NI", "J4-NI"}, "Location", "southeast");