%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2018, Louis Hawley and Wael Suleiman
%
%  Kalman Filter Based Observer for an External Force Applied to 
%  Medium-sized Humanoid Robots
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear variables;

%% Experiment #1 : Force along Y axis
test_id = 'Experiment #1 - ';
Path = 'Force_X_Z_data/';
load([Path 'Acc.mat']);
load([Path 'CoM_Pos.mat']);
load([Path 'total_force.mat']);
load([Path 'ZMP.mat']);
run('Kalman_Humanoid_Force_Observer/Kalman_filter_force_observer.m');


%% Experiment #2 : Arbitrary external force 
test_id = 'Experiment #2 - ';
Path = 'Force_Y_data/';
load([Path 'Acc.mat']);
load([Path 'CoM_Pos.mat']);
load([Path 'total_force.mat']);
load([Path 'ZMP.mat']);
run('Kalman_Humanoid_Force_Observer/Kalman_filter_force_observer.m');

%% Experiment #3 : Force in the x-z plane
test_id = 'Experiment #3 - ';
Path = 'Force_Z_data/';
load([Path 'Acc.mat']);
load([Path 'CoM_Pos.mat']);
load([Path 'total_force.mat']);
load([Path 'ZMP.mat']);
run('Kalman_Humanoid_Force_Observer/Kalman_filter_force_observer.m');

%% Experiment #4 : Force along x axis while walking
test_id = 'Experiment #4 - ';
Path = 'Force_Walk/';
load([Path 'Acc.mat']);
load([Path 'CoM_Pos.mat']);
load([Path 'total_force.mat']);
load([Path 'ZMP.mat']);
run('Kalman_Humanoid_Force_Observer/Kalman_filter_force_observer.m');
