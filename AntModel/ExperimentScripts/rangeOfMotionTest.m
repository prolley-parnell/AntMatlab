%% Trial to evaluate time per contact
% 4/04/2023 - Persie Rolley-Parnell
%% Set the environment by closing any previous figures and variables

%If errors persist and the system crashes or closes, use diary to store
%printout
    %diary lastDiaryFile

%Close any figures and clear all variables
close all;
clear;


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

% Warnings are automatically enabled
RUNTIME_ARGS.disableWarnings();

% Number of Iterations
RUNTIME_ARGS.N_TRIALS = 1;

% Sampling rate in simulated time - smaller is smoother for plotting, but
% takes longer to run (0.05 to 0.15)
RUNTIME_ARGS.RATE = 0.05;

%Plot the experiment
RUNTIME_ARGS.PLOT.ENABLE = [0 0];

% Printout saves the experiment results to an excel file, the position and pose, and cost,
% only prints out if a grasp is selected, use if data needs exporting to
% external software
RUNTIME_ARGS.PRINTOUT.ENABLE = 0;

% To save the summary tables, antTree, and runtime args in a single MAT
% file, enable. Use for internal matlab processing
RUNTIME_ARGS.RECORD.ENABLE = 1;

%% Object to sense
% % Scale is linear scale about the centre of the object, varies for all
% experiments.
RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.18;
% stl file (binary) to import
%There should be no STL imported
RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = '';

%% Ant body setup
% Body motion not required for these experiments
RUNTIME_ARGS.BODY_MOTION_ENABLE = 0;

% ------------- Antenna Motion -------------- %
% Select point-to-point or joint based control
motion_type = {'p2p'}; %{'p2p', 'joint'};

%if using p2p then p2pmode = {'GMM'};
%if using joint control then jointmode = {'mean'}
control_method = {'random'};

%If using Information gain refinement {'IG'}
refine_method = {};
            
%If using 'mean' or 'GMM' then option to set variance {'IPD', 'varinc', 'vardec', 'var=1.2'};
variance_mode = {};

antenna_control_cell = [motion_type,control_method,refine_method,variance_mode];

RUNTIME_ARGS = RUNTIME_ARGS.setAntennaControl(antenna_control_cell);


% ------------- Goal Selection Method ---------- %
% Checks all points for each of the modes included in the brackets
RUNTIME_ARGS.SENSE.MODE = {'dist'};
% Uses the centre of contact to generate a potential grasp along the PCA
% axis 
%RUNTIME_ARGS.SENSE.MODE = {'PCA'};

%Set timeout
RUNTIME_ARGS.TIMEOUT = 1000;
RUNTIME_ARGS.TRIAL_NAME = 'randp2p_motion'


%% -------- Test Specific Set Up ------------ %%
% 
[exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS);

