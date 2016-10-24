% Read data represented as a uint64
logElements = loadLogFile();

% Cast it back to double
logElements.data = typecast(uint64(logElements.data), 'double');