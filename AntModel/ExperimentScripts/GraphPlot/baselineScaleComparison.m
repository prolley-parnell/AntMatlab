%% File to run the appropriate comparisons and load code as necessary
% % Created 06/02/2023
% 
% GPC_dice = GraphPlotClass();
% GPC_plank = GraphPlotClass();
% GPC_wedge = GraphPlotClass();
% GPC_grass_seed = GraphPlotClass();
% % %Add all Experiment Folders
% experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
% WARNING : The experiments in these folders do not use the same force, so
% the qualities will be different.
% GPC_dice = GPC_dice.loadData([experimentFolder, '\dice']);
% GPC_plank = GPC_plank.loadData([experimentFolder, '\plank']);
% GPC_wedge = GPC_wedge.loadData([experimentFolder, '\wedge']);
% GPC_grass_seed = GPC_grass_seed.loadData([experimentFolder, '\grass_seed']);
% 
% %% Import all the baseline values for each shape
% baselineFile = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentScripts\baseline_evaluation_scripts\baselineQualityRangeStruct.mat';
% load(baselineFile)
% GPC_dice = GPC_dice.addBaselineQualities(diceQualityRange);
% GPC_plank = GPC_plank.addBaselineQualities(plankQualityRange);
% GPC_wedge = GPC_wedge.addBaselineQualities(wedgeQualityRange);
% GPC_grass_seed = GPC_grass_seed.addBaselineQualities(grassSeedQualityRange);
% 
% 
% % 
% save('GPC_4shape_d_p_w_gs.mat', 'GPC_dice', 'GPC_plank', "GPC_wedge", "GPC_grass_seed")

% %% 
% t = load("GPC_full_force1.mat")
% GPC_wedge.experimentDataStruct = t.GPC_wedge.experimentDataStruct;
% GPC_plank.experimentDataStruct = t.GPC_plank.experimentDataStruct;
% GPC_dice.experimentDataStruct = t.GPC_dice.experimentDataStruct;
% save('GPC_4shape_d_p_w_gs.mat', 'GPC_dice', 'GPC_plank', "GPC_wedge", "GPC_grass_seed")

%% Plot to show how the knee points are used to select percentage
experimentNameExample = "PCA_RRaP";
GPC_grass_seed.plotKneeSelection(experimentNameExample)

%%

experimentNameExample = "PCA_RRaP";
GPC_grass_seed.plotKneeSelection(experimentNameExample)

%%
[~, ~, tileGS] = GPC_grass_seed.completePaperPlot()
subtitle(tileGS, 'Grass Seed Object')

%% Extract results table per shape

[plank_resultTable] = GPC_plank.extractPlotAndCostData;
plank_maxKneeTable = GPC_plank.findMaxKnee(plank_resultTable);

[dice_resultTable] = GPC_dice.extractPlotAndCostData;
dice_maxKneeTable = GPC_dice.findMaxKnee(dice_resultTable);

[wedge_resultTable] = GPC_wedge.extractPlotAndCostData;
wedge_maxKneeTable = GPC_wedge.findMaxKnee(wedge_resultTable);

[grass_seed_resultTable] = GPC_grass_seed.extractPlotAndCostData;
grass_seed_maxKneeTable = GPC_grass_seed.findMaxKnee(grass_seed_resultTable);


%% Compare without percentage pass
[plankRank] = GPC_plank.addRank(plank_maxKneeTable);
completePlankTable = GPC_plank.addSummaryRank(plankRank);
[diceRank] = GPC_dice.addRank(dice_maxKneeTable);
completeDiceTable = GPC_dice.addSummaryRank(diceRank);
[wedgeRank] = GPC_wedge.addRank(wedge_maxKneeTable);
completeWedgeTable = GPC_wedge.addSummaryRank(wedgeRank);
[grass_seedRank] = GPC_grass_seed.addRank(grass_seed_maxKneeTable);
completeGrassSeedTable = GPC_grass_seed.addSummaryRank(grass_seedRank);


%% Compare both dice and plank to see the difference in ranking
allMeasureSummaryTable = GPC_plank.summariseTableRank(completeDiceTable, completePlankTable, completeWedgeTable, completeGrassSeedTable);
allMeasureSummaryTable.Properties.VariableNames{1} = 'Dice';
allMeasureSummaryTable.Properties.VariableNames{2} = 'Plank';
allMeasureSummaryTable.Properties.VariableNames{3} = 'Wedge';
allMeasureSummaryTable.Properties.VariableNames{4} = 'Grass Seed';


%% Order by map table not by rank
allMeasureSummaryTable = GPC_wedge.reorderRows(allMeasureSummaryTable, ["Control Method", "Search Method", "Row Index"]);


%% Copy out the max percent and knee in order of the measures summary Table

PaperTable = array2table([completeDiceTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee", "SimulationTime MaxKnee", "RealWorldTime MaxKnee"]).Variables, ...
            completePlankTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee", "SimulationTime MaxKnee", "RealWorldTime MaxKnee"]).Variables, ...
            completeWedgeTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee", "SimulationTime MaxKnee", "RealWorldTime MaxKnee"]).Variables, ...
            completeGrassSeedTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee", "SimulationTime MaxKnee", "RealWorldTime MaxKnee"]).Variables, ...
           allMeasureSummaryTable{:,["Max Max Shape Score","Max Mean Score", "SD Max Shape Score"]}]);
PaperTable.Properties.VariableNames = {'Dice K(C)', 'Dice K(P)', 'Dice tau_S', 'Dice tau_RW', 'Plank K(C)', 'Plank K(P)', 'Plank tau_S', 'Plank tau_RW', 'Wedge K(C)', 'Wedge K(P)', 'Wedge tau_S', 'Wedge tau_RW', 'Grass Seed K(C)', 'Grass Seed K(P)', 'Grass Seed tau_S', 'Grass Seed tau_RW', 'Worst Score','Worst Mean Score', 'SD of Worst Shape Score'};

PaperTable = mergevars(PaperTable, {'Dice K(C)', 'Plank K(C)', 'Wedge K(C)', 'Grass Seed K(C)'}, "NewVariableName", 'K(C)', 'MergeAsTable',true);
PaperTable = mergevars(PaperTable, {'Dice K(P)', 'Plank K(P)', 'Wedge K(P)',  'Grass Seed K(P)' }, "NewVariableName", 'K(P)', MergeAsTable=true);
PaperTable = mergevars(PaperTable, {'Dice tau_S', 'Plank tau_S', 'Wedge tau_S' , 'Grass Seed tau_S'}, "NewVariableName", 'tau S', MergeAsTable=true);
PaperTable = mergevars(PaperTable, {'Dice tau_RW', 'Plank tau_RW', 'Wedge tau_RW', 'Grass Seed tau_RW'}, "NewVariableName", 'tau RW', MergeAsTable=true);

PaperTable.Properties.RowNames = allMeasureSummaryTable.Properties.RowNames;



%% Convert paper table in to a coherent plot

GPC_plank.plotPaperRank(PaperTable)

%%
GPC_plank.plotPaperKnee(PaperTable)

%%
GPC_plank.plotPaperKneeV2(PaperTable)
