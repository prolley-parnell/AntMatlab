% %% Experiment evaluation for ROBIO Paper August 2023
experimentPath = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\';
% %Save the experiment data
% GPC_dice = GraphPlotClass();
% GPC_plank = GraphPlotClass();
% GPC_wedge = GraphPlotClass();
% GPC_grass_seed = GraphPlotClass();
% %Add all Experiment Folders
%experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\cppRemoteParallelFunction';
% GPC_dice = GPC_dice.loadData([experimentFolder, '\dice']);
% GPC_plank = GPC_plank.loadData([experimentFolder, '\plank']);
% GPC_wedge = GPC_wedge.loadData([experimentFolder, '\wedge']);
% GPC_grass_seed = GPC_grass_seed.loadData([experimentFolder, '\grass_seed']);

%% Calculate the baselines for the shapes
%run(experimentPath + 'ExperimentScripts/baseline_evaluation_scripts/allShapeBaselineScript.m')

%% Import all the baseline values for each shape
% baselineFile = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentScripts\baseline_evaluation_scripts\baselineQualityRangeStruct.mat';
% load(baselineFile)
% GPC_dice = GPC_dice.addBaselineQualities(diceQualityRange);
% GPC_plank = GPC_plank.addBaselineQualities(plankQualityRange);
% GPC_wedge = GPC_wedge.addBaselineQualities(wedgeQualityRange);
% GPC_grass_seed = GPC_grass_seed.addBaselineQualities(grassSeedQualityRange);
% GPC_plank = GPC_plank.renameTableColumns;
% GPC_dice = GPC_dice.renameTableColumns;
% GPC_wedge = GPC_wedge.renameTableColumns;
% GPC_grass_seed = GPC_grass_seed.renameTableColumns;
% 
% % 
% save('GPC_4shape_ROBIO_v2.mat', 'GPC_dice', 'GPC_plank', "GPC_wedge", "GPC_grass_seed")

%%
%Load experiment store
load('C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentScripts\GraphPlot\GPC_4shape_ROBIO_v2.mat')

%% Compare dice to baseline (amount of contacts fixed in the code)
GPC_dice.plotGraspBaselineComparison("all", 1);

%Plot the dice without excluding any failed grasps from the actual grasps
GPC_dice.plotGraspBaselineComparison("all", 0);

%% Plot the comparison of each shape
dice_percent_ax = GPC_dice.plotGraspBaselineComparison("all", 1);
plank_percent_ax = GPC_plank.plotGraspBaselineComparison("all", 1);
grass_seed_percent_ax = GPC_grass_seed.plotGraspBaselineComparison("all", 1);
wedge_percent_ax = GPC_wedge.plotGraspBaselineComparison("all", 1);
axCell = {dice_percent_ax, plank_percent_ax,  wedge_percent_ax, grass_seed_percent_ax};
%%
GPC_dice.plotCombinedPercent(axCell{:})

%% Show the refined shape baselines overlaid
%measureName = "Volume";
measureName = "all";
[diceBaselineArray, dice_bl_pass, dice_bl_fail] = GPC_dice.fetchBaseline(measureName, "true");
[plankBaselineArray, plank_bl_pass, plank_bl_fail] = GPC_plank.fetchBaseline(measureName, "true");
[wedgeBaselineArray, wedge_bl_pass, wedge_bl_fail] = GPC_wedge.fetchBaseline(measureName, "true");
[grassSeedBaselineArray, grassSeed_bl_pass, grassSeed_bl_fail] = GPC_grass_seed.fetchBaseline(measureName, "true");
shapeNames = ["Dice", "Plank", "Wedge", "Grass Seed"];
baselineArray = {diceBaselineArray, plankBaselineArray, wedgeBaselineArray, grassSeedBaselineArray};
percentPass = {(dice_bl_pass{:}/(dice_bl_pass{:}+dice_bl_fail{:})),...
                (plank_bl_pass{:}/(plank_bl_pass{:}+plank_bl_fail{:})),...
                (wedge_bl_pass{:}/(wedge_bl_pass{:}+wedge_bl_fail{:})),...
                (grassSeed_bl_pass{:}/(grassSeed_bl_pass{:}+grassSeed_bl_fail{:}))};
GPC_grass_seed.plotShapeBaselineComparisonHistogram(shapeNames, measureName, baselineArray, percentPass)


%% Plot the percent of successful grasps on all shapes across all methods as the number of contacts increases
experimentName = ["PCA_CGMMC_fixvar", "PCA_RRaP", "PCA_RSS"];
percentSuccess_diceAx = GPC_dice.plotPercentSuccess(experimentName, percentPass{1});
percentSuccess_plankAx = GPC_plank.plotPercentSuccess(experimentName, percentPass{2});
percentSuccess_wedgeAx = GPC_wedge.plotPercentSuccess(experimentName, percentPass{3});
percentSuccess_grassSeedAx = GPC_grass_seed.plotPercentSuccess(experimentName, percentPass{4});

GPC_dice.plotCombinedPercentAllContact(percentSuccess_diceAx, percentSuccess_plankAx, percentSuccess_wedgeAx, percentSuccess_grassSeedAx)



%% Compare the different number of contacts across all shapes for 1 method
nContacts = [19, 70];
GPC_input = [GPC_dice, GPC_plank, GPC_wedge, GPC_grass_seed];
experimentName = ["PCA_CGMMC_fixvar", "PCA_RRaP", "PCA_RSS"];
measureNames = [];
GPC_wedge.plotAllShapeNContactHistogram(GPC_input, experimentName, nContacts);

