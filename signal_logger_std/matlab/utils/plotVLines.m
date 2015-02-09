function plotVLines(timeArr, times, colors, active)
% function plotVLines(timeArr, times, colors, active)
%
% Draws vertical line on the current axes at specified times defined by
% 'times', but only if active is true or missing

if (~active && nargin >= 4)
    % not active
    return
end

dt = timeArr(2)-timeArr(1);
idx = [];
for i=1:length(times)
    idx(i) = find(timeArr>=times(i)-dt/2,1,'first');
end

colorArr = cell(1,length(idx));
if (length(colors) == 1)
    for i=1:length(idx)
        colorArr{i} = colors;
    end   
elseif (length(colors)==length(idx))
    colorArr = colors;
else
   error('The length of colors should be either one or equal to the times')
end

% finally, plot lines
vline(timeArr(idx), colorArr);
