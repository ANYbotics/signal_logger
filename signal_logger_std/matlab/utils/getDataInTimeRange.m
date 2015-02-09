function [time, data, idx] = getDataInTimeRange(time, data, tStart, tEnd)
% get time range
idx = find(time >= tStart);
idxStart = idx(1);
time = time(idx);
data = data(idx,:);
idx = find(time <= tEnd);
idxEnd = idx(end);
time = time(idx);
data = data(idx,:);
idx = [idxStart:1:idxStart-1+idxEnd]';