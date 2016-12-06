function [logElements, timeSyncOffset] = loadLogFile(fname)
% [logElements] = loadLogFile(fname)
%
% This function loads a log data file and stores it into a vector of
% structs. 
%
% inputs:
%       fname: logfile name (if non/invalid -> get path over ui)
% outputs:
%   logElements: struct containing log element properties
%       name:    name of the log element
%       noBytes: number of bytes of a single data point
%       noData:  number of logged data points
%       divider: determines the collect freq. of the element
%                (update_freq/divider)
%       isBufferLooping: determines whether the buffer is looping, time has
%                        to be matched in inverse manor
%       data:    uint64 vector containing the data (typecast this to
%                correct type)
%       time:    time vector matching a time to every data element. Left
%                empty by this function
%
% Gabriel Hottiger, October 2016

% read in the file name
if ~exist('fname') || isempty(fname),
	[fname, pathname] = uigetfile('*','Select Data File');
	if (fname == 0),
		return;
	end;
	% concatenate pathname and filename and open file
	fname=strcat(pathname, fname);
end;

% open as little-endian ("ieee-be" for big endian)
fid=fopen(fname, 'r', 'ieee-le');
if fid == -1,
	return;
end;

% skip comments
[ fid, currentLine ] = skipComments(fid, false);

% read time synchronization offset
timeSyncOffset = str2double(currentLine);

% skip comments
[ fid, currentLine ] = skipComments(fid, false);

% read number of elements
noElements = str2double(currentLine);

% skip comments
fid = skipComments(fid, true);

% read header
header = textscan(fid,'%s %d %d %d %d', noElements);

% skip header comments
fid = skipComments(fid, true);

% initialize log elements
logElements = repmat(struct('name', '', 'noBytes', 0, 'noData', 0, ...
                            'divider', 0, 'isBufferLooping', 0, 'data', []) , noElements, 1 );
for i=1:noElements
    logElements(i).name = header{1}(i);
    logElements(i).noBytes = header{2}(i);
    logElements(i).noData = header{3}(i);
    logElements(i).divider = header{4}(i);
    logElements(i).isBufferLooping = header{5}(i);
    logElements(i).data = typecast( fread(fid, logElements(i).noData ,...
        strcat('*uint', num2str( 8*logElements(i).noBytes ) ) ), 'uint64');
    logElements(i).time = struct('seconds', [], 'nanoseconds', []);
end

fclose(fid);
end