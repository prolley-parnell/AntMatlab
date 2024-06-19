%% Set the environment by closing any previous figures and variables
close all;
clear;

addpath('MotionControl','Body', 'Sense', 'Environment', 'urdf', 'Grasp', 'DataProcess');

rng('shuffle');

RUNTIME_ARGS = RuntimeArgs();

% Warnings are automatically enabled
RUNTIME_ARGS.disableWarnings();

%% ----------- Experiment Set up
% Number of Iterations
RUNTIME_ARGS.N_TRIALS = 100;
RUNTIME_ARGS.PLOT.ENABLE = [0 0];

RUNTIME_ARGS.PRINTOUT.ENABLE = 0;
RUNTIME_ARGS.RECORD.ENABLE = 1;


% Weights
weights.Neck = 0.5;
weights.Antenna = 0.1;
weights.Mandible = 0.4;

%% RUN TRIAL FIXED Mand GM %%

RUNTIME_ARGS.TRIAL_NAME = 'GM_MAND_DIST';

% ------------- Antenna Motion -------------- %
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.MODE = "GM";
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.VAR = 0.7;


[exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS);

%Find total motion cost

[resultsTable, meanForAllTrialsMANDVAR] = fileHandler.evaluateFolder(fileHandler.mat_path, weights);

save([fileHandler.script_folder, '\evaluation.mat'], 'resultsTable', 'meanForAllTrialsMANDVAR')

meanForAllTrialsMANDVAR

%% RUN TRIAL FIXED SPACE %%

RUNTIME_ARGS.TRIAL_NAME = 'FIXED_SPACE';

% ------------- Antenna Motion -------------- %
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.MODE = "fixed";
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.RANGE = [-2, 2; ...
                2, 4; ...
                0, 1];


[exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS);

%Find total motion cost


[resultsTable, meanForAllTrialsFIXEDSEARCH] = fileHandler.evaluateFolder(fileHandler.mat_path, weights);

save([fileHandler.script_folder, '\evaluation.mat'], 'resultsTable', 'meanForAllTrialsFIXEDSEARCH')

meanForAllTrialsFIXEDSEARCH
