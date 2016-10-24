% Read data represented as a uint64
logElements = loadLogFile('d_20161024_16-43-29_00028');

% Match correct time
logElements = matchTimeToData(logElements);

% Cast it back to double
logElements(3).data = typecast(uint64(logElements(3).data), 'double');