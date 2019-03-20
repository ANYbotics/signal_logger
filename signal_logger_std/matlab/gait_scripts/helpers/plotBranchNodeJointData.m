function plotBranchNodeJointData(axes_handle, data, branch_name, node_name, data_name, plot_lims, time_offset)
%PLOTBRANCHNODEJOINTDATA plots joint data to a given axes handle.
%
%   Author(s): Dario Bellicoso 7 Mar 2019
hold on; grid on;

idx_mea = evalin('base',['idx_state_joint' data_name '_' branch_name '_' node_name]);
idx_des = evalin('base',['idx_command_desJoint' data_name '_' branch_name '_' node_name]);

plot(axes_handle, data(idx_des).time - time_offset, data(idx_des).data, 'r', 'linewidth', 2);
plot(axes_handle, data(idx_mea).time - time_offset, data(idx_mea).data, 'b', 'linewidth', 2);

xlim([plot_lims(1)-time_offset plot_lims(2)-time_offset]);

xlabel('time [s]');
ylabel([node_name ' [rad]']);

[~, time_idx_min] = min(abs(data(idx_des).time - plot_lims(1)));
[~, time_idx_max] = min(abs(data(idx_des).time - plot_lims(2)));

disp(['branch:    ' branch_name]);
disp(['node:      ' node_name]);
disp(['data type: ' data_name]);

minDes = min(data(idx_des).data(time_idx_min:time_idx_max));
maxDes = max(data(idx_des).data(time_idx_min:time_idx_max));
minMea = min(data(idx_mea).data(time_idx_min:time_idx_max));
maxMea = max(data(idx_mea).data(time_idx_min:time_idx_max));

fprintf('des min %.2f\n',minDes);
fprintf('des max %.2f\n',maxDes);
fprintf('mea min %.2f\n',minMea);
fprintf('mea max %.2f\n',maxMea);
disp(' ');

end