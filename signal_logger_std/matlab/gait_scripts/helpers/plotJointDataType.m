function plotJointDataType(data_type_name, logElements, branch_names, ...
                           node_names, plot_lims, time_offset)
%PLOTJOINTDATATYPE sets up plotting joint data.
%
%   Author(s): Dario Bellicoso 7 Mar 2019
figure('NumberTitle', 'off', 'Name', ['Joint ' data_type_name]);

num_cols = length(branch_names);
num_rows = length(node_names);
data_type = data_type_name;

for k_branch=1:num_cols
    start_id = k_branch;
    branch_name = branch_names{k_branch};
    for k_joint=1:num_rows
        plotBranchNodeJointData(subplot(num_rows,num_cols,start_id),...
                         logElements, branch_name, node_names{k_joint}, ...
                         data_type, plot_lims, time_offset);
        if k_joint~=num_rows
          set(gca,'XTickLabel',[]);
          set(gca,'XLabel',[]);
        end
        
        if k_branch~=1
          set(gca,'YLabel',[]);
        end
        
        if k_joint==1 && k_branch==num_cols
          legend('des','meas');
        end
        
        numxticks = 5;
        xtick_step = (plot_lims(2) - plot_lims(1))/(numxticks-1);
        set(gca,'XTick',(plot_lims(1)-time_offset):xtick_step:(plot_lims(2)-time_offset));
        
        numyticks = 3;
        yticks = get(gca, 'YTick');
        ytickmax = yticks(end);
        ytickmin = yticks(1);
        ytickstep = (ytickmax-ytickmin)/(numyticks-1);
        set(gca,'YTick',ytickmin:ytickstep:ytickmax);
        
        start_id = start_id + num_cols;
    end
end

end
