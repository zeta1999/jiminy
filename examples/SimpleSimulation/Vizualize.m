% Import the data
logFull = readmatrix(fullfile(pwd,'build','log.csv'));

% Convert to output type
log = logFull;
N = size(log,1);
log = [log(:,1),log(:,9:8+12),log(:,2:8)];

%Clear temporary variables
clear opts N

%%
ExoTools.visualizeExoSimulink(log,0.412,0.403,'asIs')