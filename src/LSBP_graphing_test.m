close all
clear all
clc
target_time = 10;
tar_A = [0.185;-0.185;0.185];
tar_B = [0.185;0.170;0.070];
tar_C = [0.185;0;0.24];
phiA = -pi/4;
phiB = atan2(0.17,0.185);
T_A = [[0,-sin(phiA),-cos(phiA),0.185];
    [0,cos(phiA),-sin(phiA),-0.185];
    [1,0,0,0.185];
    [0,0,0,1]];
T_B = [[0,-sin(phiB),-cos(phiB),0.185];
    [0,cos(phiB),-sin(phiB),0.170];
    [1,0,0,0.07];
    [0,0,0,1]];
T_C = [[0,0,-1,0.185];
    [0,1,0,0];
    [1,0,0,0.24];
    [0,0,0,1]];
initial_pos = tar_C;
target_pos = tar_B;
buffer_percent = 0.10;
buffer_time = target_time * buffer_percent;
current_state = [initial_pos;0;0;0;0;0;0]; %x,y,z,vx,vy,vz,ax,ay,az
pos_arr = [];
vel_arr = [];
acc_arr = [];
dt = 0.01;

% Find target metrics
distance = target_pos - initial_pos;
target_vel = distance/((1-buffer_percent) * target_time);
target_acc = target_vel/buffer_time;
iterations = target_time/dt;

for i = 0:iterations
    time = i*dt;


    if (time < buffer_time)
        current_state(7:9) = target_acc;
        current_state(4:6) = current_state(4:6) + current_state(7:9) * dt;
        current_state(1:3) = current_state(1:3) + current_state(4:6) * dt;
    elseif (time > target_time - buffer_time)
        current_state(7:9) = -target_acc;
        current_state(4:6) = current_state(4:6) + current_state(7:9) * dt;
        current_state(1:3) = current_state(1:3) + current_state(4:6) * dt;
    else
        current_state(7:9) = 0;
        current_state(4:6) = current_state(4:6) + current_state(7:9) * dt;
        current_state(1:3) = current_state(1:3) + current_state(4:6) * dt;
    end
   
   pos_arr(i+1,1:3) = current_state(1:3);
   vel_arr(i+1,1:3) = current_state(4:6);
   acc_arr(i+1,1:3) = current_state(7:9);
end

% Create a figure window
figure;

% Plot position data (first row)
subplot(3, 3, 1);  % 3 rows, 3 columns, first subplot
plot(pos_arr(:,1));
ylim([min(pos_arr(:,1))-0.005,max(pos_arr(:,1))+0.005])
xlim([0,iterations])
title('Position X');

subplot(3, 3, 2);  % 3 rows, 3 columns, second subplot
plot(pos_arr(:,2));
ylim([min(pos_arr(:,2))-0.005,max(pos_arr(:,2))+0.005])
xlim([0,iterations])
title('Position Y');

subplot(3, 3, 3);  % 3 rows, 3 columns, third subplot
plot(pos_arr(:,3));
ylim([min(pos_arr(:,3))-0.005,max(pos_arr(:,3))+0.005])
xlim([0,iterations])
title('Position Z');

% Plot velocity data (second row)
subplot(3, 3, 4);  % 3 rows, 3 columns, fourth subplot
plot(vel_arr(:,1));
ylim([min(vel_arr(:,1))-0.005,max(vel_arr(:,1))+0.005])
xlim([0,iterations])
title('Velocity X');

subplot(3, 3, 5);  % 3 rows, 3 columns, fifth subplot
plot(vel_arr(:,2));
ylim([min(vel_arr(:,2))-0.005,max(vel_arr(:,2))+0.005])
xlim([0,iterations])
title('Velocity Y');

subplot(3, 3, 6);  % 3 rows, 3 columns, sixth subplot
plot(vel_arr(:,3));
ylim([min(vel_arr(:,3))-0.005,max(vel_arr(:,3))+0.005])
xlim([0,iterations])
title('Velocity Z');

% Plot acceleration data (third row)
subplot(3, 3, 7);  % 3 rows, 3 columns, seventh subplot
plot(acc_arr(:,1));
ylim([min(acc_arr(:,1))-0.005,max(acc_arr(:,1))+0.005])
xlim([0,iterations])
title('Acceleration X');

subplot(3, 3, 8);  % 3 rows, 3 columns, eighth subplot
plot(acc_arr(:,2));
ylim([min(acc_arr(:,2))-0.005,max(acc_arr(:,2))+0.005])
xlim([0,iterations])
title('Acceleration Y');

subplot(3, 3, 9);  % 3 rows, 3 columns, ninth subplot
plot(acc_arr(:,3));
ylim([min(acc_arr(:,3))-0.005,max(acc_arr(:,3))+0.005])
xlim([0,iterations])
title('Acceleration Z');