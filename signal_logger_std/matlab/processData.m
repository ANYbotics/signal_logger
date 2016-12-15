% Read data represented as a uint64
[logElements, timeSyncOffset] = loadLogFile();

% Match correct time
logElements = matchTimeToData(logElements, timeSyncOffset);

% Cast it back to double
logElements(3).data = typecast(uint64(logElements(3).data), 'single');
logElements(4).data = typecast(uint64(logElements(4).data), 'single');
logElements(5).data = typecast(uint64(logElements(5).data), 'single');
