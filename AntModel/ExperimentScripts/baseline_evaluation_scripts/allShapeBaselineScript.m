%% Find Shape Baseline
% Uses predefined function to generate all possible grasps then evaluate
% them

%% Add folders to path
close all;
clear;
modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\';
scriptFolder = pwd;

cd(modelFolder)

addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
    'urdf', 'Grasp', 'ExperimentOutput', 'MainScript', 'ToolClasses', 'ExperimentScripts', 'ExperimentScripts\baseline_evaluation_scripts\');

rng('shuffle');

%%

[diceQualityRange] = findBaselineShapeFunction('./Environment/12_sided_tiny_shape.stl', 0.18);
[grassSeedQualityRange] = findBaselineShapeFunction('./Environment/Grass Seed 2pt8 v3.stl', 0.8);
[plankQualityRange] = findBaselineShapeFunction('./Environment/plank.stl', 0.045);
[wedgeQualityRange] = findBaselineShapeFunction('./Environment/Wedge V1.stl', 0.4);

%%
save('./ExperimentScripts/baseline_evaluation_scripts/baselineQualityRangeStruct.mat', 'diceQualityRange', "wedgeQualityRange", "plankQualityRange", "grassSeedQualityRange")


%%
cd(scriptFolder)