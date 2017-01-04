% Read data represented as a uint64
[logElements, timeSyncOffset] = loadLogFile();

% Match correct time
logElements = matchTimeToData(logElements, timeSyncOffset);

% Cast it back to double
logElements(3).data = typecast(logElements(3).data, 'double');
logElements(4).data = typecast(logElements(4).data, 'double');
logElements(5).data = typecast(logElements(5).data, 'double');
