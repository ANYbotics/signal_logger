% Read data
[logElements, timeSyncOffset] = loadLogFile();

% Match correct time
logElements = matchTimeToData(logElements, timeSyncOffset);
