%% Init
clear all; close all;
fNumber = 164;
processData;

%% Setup
plotDuration = logElements(1).time(end);
% static walk [16.5 18.5]
% trot [23 25]
% pace [8 9]
% dyn walk [12.5 14.5]
% run trot [16.2 17.5]

startTime = 8;
endTime = 9;
plot_lims = [0 plotDuration];

%% Joints
% branch_names = {'LF', 'RF', 'LH', 'RH'};
branch_names = {'LF'};
node_names = {'HAA', 'HFE', 'KFE'};
%plotJointDataType('Pos', logElements, branch_names, node_names, plot_lims, startTime);
plotJointDataType('Vel', logElements, branch_names, node_names, plot_lims, startTime);
%plotJointDataType('Tor', logElements, branch_names, node_names, plot_lims, startTime);

%% Forces
%branch_names_extended = {'leftFore', 'rightFore', 'leftHind', 'rightHind'};
branch_names_extended = {'leftFore'};
dim_names = {'x','y','z'};
plotForceData(logElements, branch_names_extended, dim_names, plot_lims, startTime);

%% COM_xy vs ZMP plan
figure('NumberTitle', 'off', 'Name', 'Motion Plan');
grid on;
hold on;
view(3);
plot3(logElements(idx_planner_desPosWorldToComInWorld_x).data, ...
      logElements(idx_planner_desPosWorldToComInWorld_y).data, ...
      logElements(idx_planner_desPosWorldToComInWorld_z).data);

% plot3(logElements(idx_planner_desPosWorldToPiInWorld_x).data, ...
%       logElements(idx_planner_desPosWorldToPiInWorld_y).data, ...
%       logElements(idx_planner_desPosWorldToPiInWorld_z).data);

plot3(logElements(idx_planner_desPosWorldToZmpInWorld_x).data, ...
      logElements(idx_planner_desPosWorldToZmpInWorld_y).data, ...
      logElements(idx_planner_desPosWorldToZmpInWorld_z).data);

% plot(logElements(idx_planner_desPosWorldToComInWorld_x).data, ...
%      logElements(idx_planner_desPosWorldToComInWorld_y).data);
% 
% plot(logElements(idx_planner_desPosWorldToPiInWorld_x).data, ...
%      logElements(idx_planner_desPosWorldToPiInWorld_y).data);

% plot(logElements(idx_planner_desPosWorldToComInWorld_x).time, ...
%      logElements(idx_planner_desPosWorldToComInWorld_x).data);

