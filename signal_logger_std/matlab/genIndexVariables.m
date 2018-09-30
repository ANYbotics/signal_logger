function [ ] = genIndexVariables( logElements , verbose)
%GENINDEXVARIABLES Summary of this function goes here
%   Detailed explanation goes here
% % generate index variables

if nargin < 2
 verbose = true;
end

for k=1:length(logElements)
    completeName = char(logElements(k).name);
    completeName = completeName(5:end);
    [startIndex,endIndex] = regexp(completeName ,'/');
    
    if ~isempty(endIndex)
        strippedName = completeName(endIndex(end)+1:end);
    else
        strippedName = completeName;
    end
    
    % hack
    strippedName = strrep(completeName, '/', '_');
    
    idxName = ['idx' strippedName];
    
    % Fix max name length
    if (length(idxName) > 60)
        % Try to substitute known patterns
        idxName = strrep(idxName,'position','pos');
        idxName = strrep(idxName,'Position','Pos');
        idxName = strrep(idxName,'Acceleration','Acc');
        idxName = strrep(idxName,'acceleration','acc');
        idxName = strrep(idxName,'Velocity','Vel');
        idxName = strrep(idxName,'velocity','vel');
        idxName = strrep(idxName,'Contact','Con');
        idxName = strrep(idxName,'contact','con');
        idxName = strrep(idxName,'Desired','Des');
        idxName = strrep(idxName,'desired','des');
        idxName = strrep(idxName,'Linear','Lin');
        idxName = strrep(idxName,'linear','lin');
        idxName = strrep(idxName,'torso_control_gait_container','tcgc');
        idxName = strrep(idxName,'estimated','est');
    end    
    
    if(verbose)
        disp([num2str(k) ' ' idxName]);
    end
    evalin('base', [' global ' idxName]);
    evalin('base', [idxName '=' num2str(k) ';']);
end

end

