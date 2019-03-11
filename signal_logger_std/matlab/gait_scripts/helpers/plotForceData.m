function plotForceData(logElements, branch_names, dim_names, plot_lims, time_offset)
%PLOTFORCEDATA sets up plotting force data.
%
%   Author(s): Dario Bellicoso 7 Mar 2019

figure('NumberTitle', 'off', 'Name', 'Reaction forces','Position',[500,500,500,300]);

num_cols = length(branch_names);
num_rows = length(dim_names);

for k_branch=1:num_cols
    start_id = k_branch;
    branch_name = branch_names{k_branch};
    for k_dim=1:num_rows
        plotBranchForce(subplot(num_rows,num_cols,start_id), logElements, ...
                        branch_name, dim_names{k_dim}, plot_lims, time_offset);
                      
        if k_dim~=num_rows
          set(gca,'XTickLabel',[]);
          set(gca,'XLabel',[]);
        end
        
        if k_branch~=1
          set(gca,'YLabel',[]);
        end
       
        % Set y limits.
        plotdata = get(gca,'Children');
        y1data = get(plotdata(1),'YData');
        y2data = get(plotdata(2),'YData');
        maxy = max([y1data y2data]);
        miny = min([y1data y2data]);
        yrange = maxy-miny;
        ylim([miny-0.1*yrange maxy+0.1*yrange]);

        % Set x ticks.
        numxticks = 5;
        xtick_step = (plot_lims(2) - plot_lims(1))/(numxticks-1);
        set(gca,'XTick',(plot_lims(1)-time_offset):xtick_step:(plot_lims(2)-time_offset));

        % Set y ticks.
        numyticks = 3;
        yticks = get(gca, 'YTick');
        ytickmax = yticks(end);
        ytickmin = yticks(1);
        ytickstep = (ytickmax-ytickmin)/(numyticks-1);
        set(gca,'YTick',ytickmin:ytickstep:ytickmax);

        start_id = start_id + num_cols;
    end
end
