
modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel';
scriptFolder = pwd;
cd(modelFolder)

addpath('Environment', 'Grasp', 'ExperimentOutput', 'ToolClasses', 'ExperimentScripts', 'ExperimentScripts\GraphPlot');

cd(scriptFolder)
newForce = 1;

% load the results mat
experimentResultPath = [modelFolder, '\ExperimentScripts\GraphPlot\GPC_full_v2.mat'];
load(experimentResultPath);

% Make copy of GPC
GPC_dice_f1 = GPC_dice;
GPC_plank_f1 = GPC_plank;
GPC_wedge_f1 = GPC_wedge;
    
%% Load a copy of the environment and runtime args
%runtimeArgsPath = [GPC_dice.experimentPath{1},'\2_contact_pts_25-02-23_06-23\mat-files\trial_1.mat'];
runtimeArgsPath = [GPC_plank.experimentPath{1},'\2_contact_pts_25-02-23_18-49\mat-files\trial_1.mat'];
%runtimeArgsPath = [GPC_wedge.experimentPath{1},'\2_contact_pts_26-02-23_05-34\mat-files\trial_1.mat'];

load(runtimeArgsPath, 'setupRuntimeArgs');

runtimeArgs_f = setupRuntimeArgs;
runtimeArgs_f.GRASP.FORCE = newForce;

env = CollisionObjects(runtimeArgs_f);

% Set up grasp quality evaluator
gE = graspEvaluator(runtimeArgs_f, 0.84);
nExperiment = 16;


for n = 1:nExperiment
    nContact = 39;
    for m = 1:nContact
            %%Find the right table
            sGT = GPC_plank.experimentDataStruct(n).trialData(m).senseGoalTable;
            nTrial = size(sGT, 1);
            for j = 1:nTrial
                tempGoal = goalStruct();
                tempGoal = tempGoal.fromTable(sGT(j,:));

                graspQuality = gE.evaluateGoal(tempGoal, env);
                tableQuality = graspQuality.convert2table();
                GPC_plank_f1.experimentDataStruct(n).trialData(m).senseGoalTable(j,["Volume", "Epsilon"]) = tableQuality(:,["Volume", "Epsilon"]);
            end
    end
end


%%
% GPC_wedge = GPC_wedge_f1;
% GPC_dice = GPC_dice_f1;
% GPC_plank = GPC_plank_f1;
% save([modelFolder, '\ExperimentScripts\GraphPlot\GPC_full_force1.mat'], "GPC_dice", "GPC_wedge", "GPC_plank");
% 




