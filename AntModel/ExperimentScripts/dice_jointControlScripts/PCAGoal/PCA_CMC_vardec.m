%% PCA_CMC_vardec
%07/07/2023 - Persie Rolley-Parnell
%% Set the environment by closing any previous figures and variables

%If errors persist and the system crashes or closes, use diary to store
%printout
%diary lastDiaryFile

%Close any figures and clear all variables
close all;
clear;


%Fill in the char array with the folder location of AntModel
modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\AntMatlab\AntModel\';
%Save the current script location
scriptFolder = pwd;
cd(modelFolder)

%Add the subfolders within AntModel to the path
addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
    'urdf', 'Grasp', 'ExperimentOutput', 'ExperimentScripts', 'MainScript', 'ToolClasses');

%Set the seed for any random number generation to be according to the
%current time
defaultStream = RandStream('mrg32k3a', 'Seed', 'shuffle');

%% Initialise the Model Property handler - RuntimeArgs

RUNTIME_ARGS = RuntimeArgs();

% Warnings are automatically enabled
RUNTIME_ARGS.disableWarnings();

% Number of Iterations
RUNTIME_ARGS.N_TRIALS = 40;

% Sampling rate in simulated time - smaller is smoother for plotting, but
% takes longer to run (0.05 to 0.15)
RUNTIME_ARGS.RATE = 0.05;

% Printout saves the experiment results to an excel file, the position and pose, and cost,
% only prints out if a grasp is selected, use if data needs exporting to
% external software
RUNTIME_ARGS.PRINTOUT.ENABLE = 0;

% To save the summary tables, antTree, and runtime args in a single MAT
% file, enable. Use for internal matlab processing
RUNTIME_ARGS.RECORD.ENABLE = 1;

%% Object to sense
% Scale is linear scale about the centre of the object, varies for all
% experiments.
RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.18;
% stl file (binary) to import
RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = './Environment/12_sided_tiny_shape.stl';

%% Ant body setup
% Body motion not required for these experiments
RUNTIME_ARGS.BODY_MOTION_ENABLE = 0;

% ------------- Antenna Motion -------------- %
% Select point-to-point or joint based control
motion_type = {'joint'}; %{'p2p', 'joint'};

%if using p2p then p2pmode = {'GMM'};
%if using joint control then jointmode = {'mean'}
control_method = {'mean'};

%If using Information gain refinement {'IG'}
refine_method = {};

%If using 'mean' or 'GMM' then option to set variance {'varinc', 'vardec', 'var=1.2'};
variance_mode = {'vardec'};

antenna_control_cell = [motion_type,control_method,refine_method,variance_mode];

RUNTIME_ARGS = RUNTIME_ARGS.setAntennaControl(antenna_control_cell);


% ------------- Goal Selection Method ---------- %
% Checks all points for each of the modes included in the brackets
%RUNTIME_ARGS.SENSE.MODE = {'dist','align'};
% Uses the centre of contact to generate a potential grasp along the PCA
% axis [NOT IMPLEMENTED]
RUNTIME_ARGS.SENSE.MODE = {'PCA'};

%% -------- Experiment Specific Set Up ------------ %%

% Experiment runs trial_n experiments per number of contact points sensed
% before goal evaluation
NumberOfPoints = [22:1:70];
nExperiment = length(NumberOfPoints);

%Structure:
% 1 - Replicate the runtime args for the number of experiments
% 2 - Assign the controlled variable to the replicated runtime args.

RUNTIME_ARGS_i = repmat(RUNTIME_ARGS, [1, nExperiment]);
experiment_name = 'dice\PCA_CMC_vardec'; %Fill in with the name of the folder

for i = 1: nExperiment
    RUNTIME_ARGS_i(i).TRIAL_NAME = [experiment_name,'\', int2str(NumberOfPoints(i)), '_contact_pts'];
    RUNTIME_ARGS_i(i) = RUNTIME_ARGS_i(i).setStoppingCriterion('n_contact', NumberOfPoints(i));
end

%Start timer for this experiment
experimentStartTime = tic;
%Disable any printed warnings for the parallel pool
delete(gcp('nocreate'))
p = parpool();
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);

R_A_i = parallel.pool.Constant(RUNTIME_ARGS_i);

disp(['Experiment started at: ', char(datetime("now"))])

parfor (n = 1:nExperiment, opts)
    if R_A_i.Value(n).INDEPENDENT
        set(defaultStream, 'Substream', n);
        RandStream.setGlobalStream(defaultStream);
    end
    [exitflag, fileHandler] = AntModelFunction(R_A_i.Value(n));
end
experimentEndTime = toc(experimentStartTime);
disp(['Experiment named: ', experiment_name, ' completed in ', num2str(experimentEndTime/3600, 3),' hours.']);

%Change back to the script folder in case of multiple scripts being run
cd(scriptFolder)