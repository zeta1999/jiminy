%% Setup the Import Options
opts = delimitedTextImportOptions("NumVariables", 38);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["tNow", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23", "x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31", "x32", "x33", "x34", "x35", "x36", "x37"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
logFull = readtable(fullfile(pwd,'build','log.csv'), opts);

% Convert to output type
log = table2array(logFull);
N = size(log,1);
log = [log(:,1),log(:,9:8+12),log(:,2:8)];

%Clear temporary variables
clear opts N

%%
ExoTools.visualizeExoSimulink(log,0.380,0.403,'asIs')