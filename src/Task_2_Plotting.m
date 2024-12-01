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