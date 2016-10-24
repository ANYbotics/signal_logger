function [ leOut ] = matchTimeToData( leIn, sTimeIdx, nsTimeIdx)
%MATCHTIMETODATA Summary of this function goes here
%   Detailed explanation goes here

% Set default indices of seconds and nanoseconds
if nargin < 3
 nsTimeIdx = 2;
end
if nargin < 2
 sTimeIdx = 1;
end

% Set out equals in
leOut = leIn;

% Check buffer type
for i=1:length(leOut)
   if(leOut(i).isBufferLooping)
    stopIdx = length(leOut(sTimeIdx).data) - mod(length(leOut(sTimeIdx).data), leOut(i).divider);
    startIdx = stopIdx - leOut(i).divider*(leOut(i).noData - 1);
    leOut(i).time.seconds = leOut(sTimeIdx).data(startIdx:leOut(i).divider:stopIdx);
    leOut(i).time.nanoseconds = leOut(nsTimeIdx).data(startIdx:leOut(i).divider:stopIdx);
   else
    stopIdx = 1 + (leOut(i).noData-1)*leOut(i).divider;
    leOut(i).time.seconds = leOut(sTimeIdx).data(1:leOut(i).divider:stopIdx);
    leOut(i).time.nanoseconds = leOut(nsTimeIdx).data(1:leOut(i).divider:stopIdx);
   end
end

end

