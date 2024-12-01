clear all
clc
close all

target_time = 10;
tar_A = [185,-185,185];
tar_B = [185;170;70];
tar_C = [185,0,240];

robot = Robot();
[pos_arr, vel_arr, acc_arr] = robot.LSPB(tar_A, tar_C, target_time);

