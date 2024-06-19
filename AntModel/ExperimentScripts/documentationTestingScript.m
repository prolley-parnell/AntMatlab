%% Documentation Testing Script

%Fill in the char array with the folder location of AntModel
modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\';
%Save the current script location
scriptFolder = pwd;
cd(modelFolder)

%Add the subfolders within AntModel to the path 
addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
    'urdf', 'Grasp', 'ExperimentOutput', 'ExperimentScripts', 'MainScript', 'ToolClasses');

%Set the seed for any random number generation to be according to the
%current time (rng does not work with parallel computing)
rng('shuffle');

%% Initialise the Model Property handler - RuntimeArgs

RUNTIME_ARGS = RuntimeArgs();

%% -------- Test Specific Set Up ------------ %%
% 
[exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS);

