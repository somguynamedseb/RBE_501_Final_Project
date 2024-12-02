clc
close all
clear variables

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

phiA = -pi/4;
T = [[0,-sin(phiA),-cos(phiA),0.185];
    [0,cos(phiA),-sin(phiA),-0.185];
    [1,0,0,0.185];
    [0,0,0,1]];

initial_guess = [0;0;0;0];
eomg = 0.001;
ev = 0.001;

[q,s] = IKinSpace(Slist,M,T,initial_guess,eomg,ev)