clc
clear variables
close all

% define variables
l1 = 0.096326;
l2 = 0.128;
l3 = 0.024;
l4 = 0.124;
l5 = 0.1334;
g = [0; 0; -9.8];

robot = Robot();

% input home configuration (rows)
M = [[0, 0, -1, l3+l4+l5]; [0, 1, 0, 0]; [1, 0, 0, l1+l2]; [0, 0, 0, 1]];

T = [[1,0,0,0.15];
    [0,1,0,0];
    [0,0,1,0.05];
    [0,0,0,1]]

% input twists at home configuration (columns)
Slist = [[0; 0; 1; 0; 0; 0], ...
         [0; 1; 0; -l1; 0; 0], ...
         [0; 1; 0; -l1-l2; 0; l3], ...
         [0; 1; 0; -l1-l2; 0; l3+l4]];

% input joint angles relative to home configuration (column)
% thetalist = [0; 1.3595; 1.3696; -2.7292];

% input joint velocities and accelerations
dthetalist = [0; 0; 0; 0];
ddthetalist = [0; 0; 0; 0];

% input Mlist
M01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.0354]; [0, 0, 0, 1]];
M12 = [[1, 0, 0, 0.003809]; [0, 1, 0, 0]; [0, 0 ,1, 0.163326]; [0, 0, 0, 1]];
M23 = [[1, 0, 0, 0.11038]; [0, 1, 0, 0]; [0, 0, 1, 0.02676]; [0, 0, 0, 1]];
M34 = [[1, 0, 0, 0.10038]; [0, 1, 0, 0]; [0, 0, 1, 0.003165]; [0, 0, 0, 1]];
M45 = [[0, 0, 1, 0.06683]; [0, 1, 0, 0]; [-1, 0, 0, -0.003165]; [0, 0, 0, 1]];
Mlist = cat(3, M01, M12, M23, M34, M45)

% input Glist
Ixx1 = 0.00005359;
Iyy1 = 0.00005149;
Izz1 = 0.00002604;
Ixy1 = 0;
Iyz1 = 0;
Ixz1 = 0;
m1 = 0.11584;

Ixx2 = 0.00022943;
Iyy2 = 0.00023236;
Izz2 = 0.00005372;
Ixy2 = 0;
Iyz2 = 0;
Ixz2 = 0.00002833;
m2 = 0.143;

Ixx3 = 0.00002916;
Iyy3 = 0.00013968;
Izz3 = 0.00015097;
Ixy3 = 0;
Iyz3 = 0;
Ixz3 = 0;
m3 = 0.122;

Ixx4 = 0.00029155;
Iyy4 = 0.00020877;
Izz4 = 0.00019077;
Ixy4 = 0;
Iyz4 = 0;
Ixz4 = 0;
m4 = 0.225;

G1 =  [[Ixx1, Ixy1, Ixz1, 0, 0, 0]; [Ixy1, Iyy1, Iyz1, 0, 0, 0]; ...
    [Ixz1, Iyz1, Izz1, 0, 0, 0]; [0, 0, 0, m1, 0, 0]; ...
    [0, 0, 0, 0, m1, 0]; [0, 0, 0, 0, 0, m1]];
G2 =  [[Ixx2, Ixy2, Ixz2, 0, 0, 0]; [Ixy2, Iyy2, Iyz2, 0, 0, 0]; ...
    [Ixz2, Iyz2, Izz2, 0, 0, 0]; [0, 0, 0, m2, 0, 0]; ...
    [0, 0, 0, 0, m2, 0]; [0, 0, 0, 0, 0, m2]];
G3 =  [[Ixx3, Ixy3, Ixz3, 0, 0, 0]; [Ixy3, Iyy3, Iyz3, 0, 0, 0]; ...
    [Ixz3, Iyz3, Izz3, 0, 0, 0]; [0, 0, 0, m3, 0, 0]; ...
    [0, 0, 0, 0, m3, 0]; [0, 0, 0, 0, 0, m3]];
G4 =  [[Ixx4, Ixy4, Ixz4, 0, 0, 0]; [Ixy4, Iyy4, Iyz4, 0, 0, 0]; ...
    [Ixz4, Iyz4, Izz4, 0, 0, 0]; [0, 0, 0, m4, 0, 0]; ...
    [0, 0, 0, 0, m4, 0]; [0, 0, 0, 0, 0, m4]];
Glist = cat(3, G1, G2, G3, G4)

% input wrench at end effector
Ftip = [0; 0; 0; 0; 0; 10];

% compute
[thetalist,s]=IKinSpace(Slist,M,T,[0;0;0;0],0.001,0.001);
taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, ...
                        Ftip, Mlist, Glist, Slist)

robot.interpolate_jp(q,2000);

j_cur = robot.measure_js(0,0,1);

pause(1)
tau_vars = taulist/j_cur

