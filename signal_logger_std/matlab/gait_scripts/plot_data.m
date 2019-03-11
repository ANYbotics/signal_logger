%% Init
clear all; close all;
fNumber = 1037;
processData;

plotDuration = logElements(1).time(end);
% static walk [17 19]
% trot [23 25]

startTime = 23;
endTime = 25;
plot_lims = [startTime endTime];

%% Joints
% branch_names = {'LF', 'RF', 'LH', 'RH'};
branch_names = {'LF'};
node_names = {'HAA', 'HFE', 'KFE'};
plotJointDataType('Pos', logElements, branch_names, node_names, plot_lims, startTime);
plotJointDataType('Vel', logElements, branch_names, node_names, plot_lims, startTime);
plotJointDataType('Tor', logElements, branch_names, node_names, plot_lims, startTime);

%% Forces
branch_names_extended = {'leftFore', 'rightFore', 'leftHind', 'rightHind'};
% branch_names_extended = {'leftFore'};
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

