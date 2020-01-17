% Complete PATH
addpath("../helper_methods", "--begin")
addpath("../utils", "--begin")

% Set example data
useExampleFile = true;
fName = "testData"

% Get filename from directory if we are not using the example data
if ~useExampleFile
  fNumber = 5;
  fName = getFilenameFromNumber(fNumber, ['/home/', getenv('LOGNAME'), '/.ros']);
  fprintf(['\nGot filename: ', fName, ' from number: ', num2str(fNumber)]);
end

% Read data
logElements = loadLogFile(fName);
fprintf(['\n\nLoaded data from binary file:', fName]);

% Generate Index Variables
verbose = false;
genIndexVariables(logElements, verbose);
fprintf('\n\nGenerated indices for the log elements!\n');

if useExampleFile
  % Plot values A, B and C
  myPlot = figure();
  title('Example Plot');

  subplot(3,1,1);
  grid on; hold on;
  xlabel('time [s]'); ylabel('A');
  plotDataWithName(logElements, idx_A, {'linestyle','-.','color','r'});

  subplot(3,1,2);
  grid on; hold on;
  xlabel('time [s]'); ylabel('B');
  plotDataWithName(logElements, idx_B, {'linestyle','-.','color','r'});

  subplot(3,1,3);
  grid on; hold on;
  xlabel('time [s]'); ylabel('C');
  plotDataWithName(logElements, idx_C, {'linestyle','-.','color','r'});
else
  fprintf("\nGood to go, go ahead and plot by:\n\n\tplotDataWithName(logElements, idx_*, {'color', 'r'})\n\n");
end