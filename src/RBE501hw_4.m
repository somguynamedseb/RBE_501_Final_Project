clear
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
M = [[0,0,-1,L2+L3+L4];
    [0,1,0,0];
    [1,0,0,L0+L1];
    [0,0,0,1]];

mL01 = 95.54;
mL12 = 143;
mL23 = 122;
mL34 = 144.13;
m01 = [[1,0,0,0];
    [0,1,0,0];
    [0,0,1,81.17];
    [0,0,0,1]];
m12 = [[1,0,0,4.64];
    [0,1,0,-0.25];
    [0,0,1,199.13];
    [0,0,0,1]];
m23 = [[0,0,1,116.32];
    [0,1,0,-0.07];
    [1,0,0,225.08];
    [0,0,0,1]];
m34 = [[0,0,1,294.44];
    [0,1,0,0]
    [1,0,0,223.23];
    [0,0,0,1]];
m45 = [[0,0,1,294.44];
    [0,1,0,0];
    [1,0,0,223.23];
    [0,0,0,1]];
% Mlist = [m01;m12;m23;m34]
Mlist = cat(3, m01,m12,m23,m34,m45);
T = [[1,0,0,0.15];
    [0,1,0,0];
    [0,0,1,0.05];
    [0,0,0,1]]

[q,s]=IKinSpace(Slist,M,T,[0;0;0;0],0.001,0.001);
rad2deg(q)


Ib1 = [[662852.51,0.00,1.94];
	[0.00,660317.00,5791.35];
	[1.94,5791.35,19303.66]];
MI1 = mL01*eye(3);
G1= [[Ib1,zeros(3)];
    [zeros(3),MI1]]

Ib2 = [[5903004.92,2035.64,163856.50];
    [2035.64,5910552.58,-1761.21];
    [163856.50,-1761.21,58720.18]];
MI2 = mL12*eye(3);
G2= [[Ib2,zeros(3)];
    [zeros(3),MI2]]

Ib3 = [[6210233.81,3448.55,3186600.79];
	[3448.55,7981489.43,-2740.42];
	[3186600.79,-2740.42,1812002.21]];
MI3 = mL23*eye(3);
G3 = [[Ib3,zeros(3)];
    [zeros(3),MI3]]

Ib4 = [[7616550.26,-3438.75,7037390.99];
	[-3438.75,14263443.45,-3656.78];
	[7037390.99	,-3656.78,6822124.25]];
MI4 = mL34*eye(3);
G4 = [[Ib4,zeros(3)];
    [zeros(3),MI4]]
% Glist = [G1;G2;G3;G4]
Glist = cat(3, G1, G2, G3, G4);

zeros_list = [0;0;0;0];
InverseDynamics(q,zeros_list,zeros_list,[0; 0; -9.8],[0;0;10;0;0;0],Mlist,Glist,Slist)