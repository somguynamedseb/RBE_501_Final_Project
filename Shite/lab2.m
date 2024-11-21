%% Set up robot object
robot = Robot();

%% Live model setup
robot_model = Model(robot);

%% Test dh2mat()
q1 = [0 20 0 -90];
q2 = [0 10 0 90];
T_test = robot.dh2mat(q1);

%% Test dh2fk()
T01_test = eye(4);
Q = [q1; q2]; % 2DOF robot example

T_test2 = robot.dh2fk(Q, T01_test);
size(T_test2); % check the size of the result, should be 4x4

%% Test fk3001()
j1 = [0; 0; 0; 0];
j2 = [10; 10; 10; 10]; % for report data
j3 = [-5; -10; -5; -10]; % for report data
j4 = [45; 30; 60; 90]; % for report data

T_test3001 = robot.fk3001(j1);
T_test3001_2 = robot.fk3001(j2);
T_test3001_3 = robot.fk3001(j3);
T_test3001_4 = robot.fk3001(j4);
