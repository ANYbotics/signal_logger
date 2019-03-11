function plotBranchNodeJointData(axes_handle, data, branch_name, node_name, data_name, plot_lims, time_offset)
%PLOTBRANCHNODEJOINTDATA plots joint data to a given axes handle.
%
%   Author(s): Dario Bellicoso 7 Mar 2019
hold on; grid on;

idx_des = evalin('base',['idx_state_joint' data_name '_' branch_name '_' node_name]);
idx_mea = evalin('base',['idx_command_desJoint' data_name '_' branch_name '_' node_name]);

plot(axes_handle, data(idx_des).time - time_offset, data(idx_des).data, 'r', 'linewidth', 2);
plot(axes_handle, data(idx_mea).time - time_offset, data(idx_mea).data, 'b', 'linewidth', 2);

xlim([plot_lims(1)-time_offset plot_lims(2)-time_offset]);

xlabel('time [s]')
ylabel([node_name ' [rad]']);

end