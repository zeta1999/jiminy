%% Import the data
logFull = readmatrix(fullfile(pwd,'build','log.csv'));
log = logFull;
log = [log(:,1),log(:,9:8+12),log(:,2:8)];

%% Visualize
ExoTools.visualizeExoSimulink(log,0.412,0.403,'asIs');