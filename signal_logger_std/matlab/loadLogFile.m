function [logElements] = loadLogFile(fname)
% [D,vars,freq] = loadLogFile(fname)
%
% This function converts an binary file into a Matlab matrix and
% a struct array of variable names and variable units. If fname is
% given, the file is processed immediately. If no filename is given,
% a dialog box will ask to located the file.
%
% fname (i): input file name (optional)
% D     (o): data matrix
% vars  (o): struct array containing variable names and units
% freq  (o): sampling frequency
%

% Stefan Schaal, March 2006

% read in the file name
if ~exist('fname') | isempty(fname),
	[fname, pathname] = uigetfile('*','Select Data File');
	if (fname == 0),
		return;
	end;
	% concatenate pathname and filename and open file
	fname=strcat(pathname, fname);
end;

% open as big-endian ("ieee-le" for little endian)
fid=fopen(fname, 'r', 'ieee-le');
if fid == -1,
	return;
end;

% skip header comments
currentLine = fgets(fid);
while( strcmp(currentLine(1:2), '//') | currentLine == sprintf('\n'))
    currentLine = fgets(fid);
end

% read nr of elements
noElements = str2double(currentLine);

% read header
header = textscan(fid,'%s %d %d %d %d', noElements);

% skip empty lines until data
positionInFile = ftell(fid);
currentLine = fgets(fid);
while( currentLine == sprintf('\n') )
    positionInFile = ftell(fid);
    currentLine = fgets(fid);
end
fseek(fid, positionInFile, 'bof');

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
end

fclose(fid);