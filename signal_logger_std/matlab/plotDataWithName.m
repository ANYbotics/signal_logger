function plotDataWithName(data, idx, color, varargin)

if (isempty(varargin))
    multiplier = 1;
else
    multiplier = varargin{1};
end

    plot(data(idx).time, multiplier*(data(idx).data), color{:});
end