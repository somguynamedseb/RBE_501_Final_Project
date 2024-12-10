clc
clear variables
close all
t_con = 2.0746 %% Amps per Nm
robot = Robot();

while 1
    mA = measured_js(0,0,1)
    A = mA/1000
    torque_est = A*t_con
    pause(0.5)

end