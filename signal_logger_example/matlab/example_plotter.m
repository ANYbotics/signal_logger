%% Load Data

% Get filename from directory
fNumber = 23;
fName = getFilenameFromNumber(fNumber, ['/home/', getenv('LOGNAME'), '/.ros']);
fprintf(['\nGot filename: ', fName, ' from number: ', num2str(fNumber)]);

% Read data
logElements = loadLogFile(fName);
fprintf(['\n\nLoaded data from binary file:', fName]);

% Generate Index Variables
verbose = false;
genIndexVariables(logElements, verbose);
fprintf('\n\nGenerated indices for the log elements!\n');

% Increase precision in data cursor and show index of data point
set(0,'defaultFigureCreateFcn',@(s,e)datacursorextra(s))


%% Plot 

figure
hold on
title("Example plot")
xlabel("Time [s]")
ylabel("MyLogVar")
plot(logElements(idx_ns_logVar1).systime, logElements(idx_ns_logVar1).data, '*', 'MarkerSize', 10);
plot(logElements(idx_ns_logVar2).systime, logElements(idx_ns_logVar2).data, '*', 'MarkerSize', 10); 
plot(logElements(idx_ns_logVar4).systime, logElements(idx_ns_logVar4).data, 'x', 'MarkerSize', 10);
plot(logElements(idx_ns_logVar5).systime, logElements(idx_ns_logVar5).data, 'o', 'MarkerSize', 10); 