function [varargout] = named_figure(str, id)

global g_figure_list;

start_index = 1000;

if exist('id', 'var'),
  str = sprintf('%s %i', str, id);
end

if 0 == max(size(g_figure_list))  
  h = add_new_figure(str);
else
  % search the cells
  found = false;
  for i=1:size(g_figure_list, 2)
    if strcmp(str, g_figure_list{i}.name),
      h = figure(g_figure_list{i}.handle);
      found = true;
      break;
    end
  end
  
  if ~found,
    h = add_new_figure(str);
  end
end

nout = max(nargout,1)-1;
if 1 == nout,
  varargout(1) = h;
end

% We always set this in case the figure has been created before and simply
% closed.
set(h, 'Name', str);

  function [out] = add_new_figure(str)
    index = size(g_figure_list, 2)+1;
    out = figure(start_index + index);
    g_figure_list{index}.handle = out;
    g_figure_list{index}.name = str;   
  
  end

end



