function [time, data, vars] = getDataFromFile(startNo, endNo, folder)


% load files
data_file = [];
for k=startNo:endNo
    fname = [folder 'd'];
    ss = num2str(k);
    for j=1:1:5-length(ss)
        fname = [fname '0'];
    end
    fname = [fname ss];
    [data_cur,vars,freq] = clmcplot_convert(fname);
    data_file = [data_file; data_cur];
end



% select data
idx0=find(data_file(:,1)==0); % get rid of empty entries at the end
if length(idx0) > 1
    idx = 1:1:idx0(2)-1; 
else
    idx = 1:size(data_file,1);
end

time = data_file(idx,1);
data = data_file(idx,:);

