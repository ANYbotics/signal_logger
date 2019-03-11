function plot3dDataWithName(data, idx_x, idx_y, idx_z, color, varargin)

if (isempty(varargin))
    multiplier = 1;
else
    multiplier = varargin{1};
end

multiplier = 1;

    plot3(multiplier*(data(idx_x).data), ...
          multiplier*(data(idx_y).data), ...
          multiplier*(data(idx_z).data), ...
          color{:});
    
end