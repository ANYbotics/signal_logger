function plotBranchForce(fig_handle, data, branch_name, dim_name, plot_lims, time_offset)
%PLOTBRANCHFORCE plots force data to a given axes handle.
%
%   Author(s): Dario Bellicoso 7 Mar 2019
hold on; grid on;

idx_des = evalin('base',['idx_loco_' branch_name '_desiredForceAtEEInWorldFrame_' dim_name]);
idx_mea = evalin('base',['idx_loco_' branch_name '_forceAtEEInWorldFrame_' dim_name]);
% weight = 35*9.81;
normalization_factor = 1;%/weight;
plot(fig_handle, data(idx_des).time-time_offset, -normalization_factor*data(idx_des).data, 'r', 'linewidth', 2);
plot(fig_handle, data(idx_mea).time-time_offset,  normalization_factor*data(idx_mea).data, 'b', 'linewidth', 2);

xlim([plot_lims(1)-time_offset plot_lims(2)-time_offset]);

xlabel('time [s]')
ylabel([dim_name ' [N]']);

end