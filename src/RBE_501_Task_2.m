clear all
clc
close all

target_time = 10;
buffer_percent = 0.2;
buffer_time = target_time * buffer_percent;
dt = 0.01;
tar_A = [185,-185,185];
tar_B = [185;170;70];
tar_C = [185,0,240];




current_state = [tar_A,0,0,0,0,0,0]; %x,y,z,vx,vy,vz,ax,ay,az
pos_arr = [];
vel_arr = [];
acc_arr = [];

dist =  normalize((current_state(1:3)-tar_C));
target_vel = dist/((1-buffer_percent) * target_time);
target_acc = target_vel/buffer_time;
iterations = target_time/dt
for i = 0:iterations
    time = i*dt

    if time< buffer_time   
        current_state(7:9) = target_acc;
        current_state(4:6) = current_state(4:6) + current_state(7:9) * dt;
        current_state(1:3) = current_state(1:3) + current_state(4:6) * dt;
    elseif time> target_time - buffer_time   
        current_state(7:9) = -target_acc;
        current_state(4:6) = current_state(4:6) + current_state(7:9) * dt;
        current_state(1:3) = current_state(1:3) + current_state(4:6) * dt;
    else
        current_state(7:9) = 0;
        current_state(4:6) = current_state(4:6) + current_state(7:9) * dt;
        current_state(1:3) = current_state(1:3) + current_state(4:6) * dt;
    end
   pos_arr(i+1,1:3) = (current_state(1:3));
   vel_arr(i+1,1:3) = (current_state(4:6));
   acc_arr(i+1,1:3) = (current_state(7:9));
end



figure;

% Plot position data (first row)
subplot(3, 3, 1);  % 3 rows, 3 columns, first subplot
plot(pos_arr(:, 1));
title('Position 1');

subplot(3, 3, 2);  % 3 rows, 3 columns, second subplot
plot(pos_arr(:, 2));
title('Position 2');

subplot(3, 3, 3);  % 3 rows, 3 columns, third subplot
plot(pos_arr(:, 3));
title('Position 3');

% Plot velocity data (second row)
subplot(3, 3, 4);  % 3 rows, 3 columns, fourth subplot
plot(vel_arr(:, 1));
title('Velocity 1');

subplot(3, 3, 5);  % 3 rows, 3 columns, fifth subplot
plot(vel_arr(:, 2));
title('Velocity 2');

subplot(3, 3, 6);  % 3 rows, 3 columns, sixth subplot
plot(vel_arr(:, 3));
title('Velocity 3');

% Plot acceleration data (third row)
subplot(3, 3, 7);  % 3 rows, 3 columns, seventh subplot
plot(acc_arr(:, 1));
title('Acceleration 1');

subplot(3, 3, 8);  % 3 rows, 3 columns, eighth subplot
plot(acc_arr(:, 2));
title('Acceleration 2');

subplot(3, 3, 9);  % 3 rows, 3 columns, ninth subplot
plot(acc_arr(:, 3));
title('Acceleration 3');