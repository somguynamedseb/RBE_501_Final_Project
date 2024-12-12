close all
clear all
clc


L0 = 0.096326;
L1 = 0.128;
L2 = 0.024;
L3 = 0.124;
L4 = 0.1334;

s1 = [0,0,1,0,0,0].';
s2 = [0,1,0,-L0,0,0].';
s3 = [0,1,0,-L0-L1,0,L2].';
s4 = [0,1,0,-L0-L1,0,L2+L3].';
Slist = [s1,s2,s3,s4];

% Input Mlist
M01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.0354]; [0, 0, 0, 1]];
M12 = [[1, 0, 0, 0.003809]; [0, 1, 0, 0]; [0, 0 ,1, 0.163326]; [0, 0, 0, 1]];
M23 = [[1, 0, 0, 0.11038]; [0, 1, 0, 0]; [0, 0, 1, 0.02676]; [0, 0, 0, 1]];
M34 = [[1, 0, 0, 0.10038]; [0, 1, 0, 0]; [0, 0, 1, 0.003165]; [0, 0, 0, 1]];
M45 = [[0, 0, 1, 0.06683]; [0, 1, 0, 0]; [-1, 0, 0, -0.003165]; [0, 0, 0, 1]];
Mlist = cat(3, M01, M12, M23, M34, M45);

% Input Glist
Ixx1 = 0.00005359; % kg*m^2
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
Glist = cat(3, G1, G2, G3, G4);

M = [[0,0,-1,L2+L3+L4];
    [0,1,0,0];
    [1,0,0,L0+L1];
    [0,0,0,1]];

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

initial_guess = [0;0;0;0];
eomg = 0.001;
ev = 0.001;


[qA,s] = IKinSpace(Slist,M,T_A,initial_guess,eomg,ev);

[qB,s] = IKinSpace(Slist,M,T_B,initial_guess,eomg,ev);

qDelta = (qA-qB);
qMid = (qA-qB)+qA
Ftip = [0; 0; 0; 0; 0; 0];
taus = InverseDynamics(qMid,qDelta,[0;0;0;0],[0;0;-9.81],Ftip,Mlist,Glist,Slist)
