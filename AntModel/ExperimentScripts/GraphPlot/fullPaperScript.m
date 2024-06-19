%% File to run the appropriate comparisons and load code as necessary
% % Created 06/02/2023
% 
% GPC_dice = GraphPlotClass();
% GPC_plank = GraphPlotClass();
% GPC_wedge = GraphPlotClass();
% GPC_grass_seed = GraphPlotClass();
% % %Add all Experiment Folders
% experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
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


%% Plot to show how the knee points are used to select percentage
experimentNameExample = "PCA_RRaP";
GPC_grass_seed.plotKneeSelection(experimentNameExample)

%% Plot the full grid of experiments and their corresponding knee values
[~, ~, tilePlank] = GPC_plank.completePaperPlot()
subtitle(tilePlank, 'Plank Object')
[~, ~, tileDice] = GPC_dice.completePaperPlot()
subtitle(tileDice, 'Dice Object')
[~, ~, tileWedge] = GPC_wedge.completePaperPlot()
subtitle(tileWedge, 'Wedge Object')

%% Extract results table per shape

[plank_resultTable] = GPC_plank.extractPlotAndCostData;
plank_maxKneeTable = GPC_plank.findMaxKnee(plank_resultTable);

[dice_resultTable] = GPC_dice.extractPlotAndCostData;
dice_maxKneeTable = GPC_dice.findMaxKnee(dice_resultTable);

[wedge_resultTable] = GPC_wedge.extractPlotAndCostData;
wedge_maxKneeTable = GPC_wedge.findMaxKnee(wedge_resultTable);

%% Add percent pass
percentPass = 0.5;
plankPTable = GPC_plank.addPercentPassColumn(plank_maxKneeTable, percentPass);
dicePTable = GPC_dice.addPercentPassColumn(dice_maxKneeTable, percentPass);
wedgePTable = GPC_wedge.addPercentPassColumn(wedge_maxKneeTable, percentPass);

%% Rank percent tables
[plankRank] = GPC_plank.addRank(plankPTable);
completePlankTable = GPC_plank.addSummaryRank(plankRank);
[diceRank] = GPC_dice.addRank(dicePTable);
completeDiceTable = GPC_dice.addSummaryRank(diceRank);
[wedgeRank] = GPC_wedge.addRank(wedgePTable);
completeWedgeTable = GPC_wedge.addSummaryRank(wedgeRank);

%% Compare without percentage pass
[plankRank] = GPC_plank.addRank(plank_maxKneeTable);
completePlankTable = GPC_plank.addSummaryRank(plankRank);
[diceRank] = GPC_dice.addRank(dice_maxKneeTable);
completeDiceTable = GPC_dice.addSummaryRank(diceRank);
[wedgeRank] = GPC_wedge.addRank(wedge_maxKneeTable);
completeWedgeTable = GPC_wedge.addSummaryRank(wedgeRank);


%% Compare both dice and plank to see the difference in ranking
allMeasureSummaryTable = GPC_plank.summariseTableRank(completeDiceTable, completePlankTable, completeWedgeTable);
allMeasureSummaryTable.Properties.VariableNames{1} = 'Dice';
allMeasureSummaryTable.Properties.VariableNames{2} = 'Plank';
allMeasureSummaryTable.Properties.VariableNames{3} = 'Wedge';

%% Copy out the max percent and knee in order of the measures summary Table

PaperTable = array2table([completeDiceTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee"]).Variables, ...
            completePlankTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee"]).Variables, ...
            completeWedgeTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee"]).Variables, ...
            allMeasureSummaryTable.Variables]);
PaperTable.Properties.VariableNames = ['Dice K(C)', 'Dice K(P)', 'Plank K(C)', 'Plank K(P)', 'Wedge K(C)', 'Wedge K(P)', allMeasureSummaryTable.Properties.VariableNames];

PaperTable = mergevars(PaperTable, {'Dice K(C)', 'Plank K(C)', 'Wedge K(C)'}, "NewVariableName", 'K(C)', 'MergeAsTable',true);
PaperTable = mergevars(PaperTable, {'Dice K(P)', 'Plank K(P)', 'Wedge K(P)'}, "NewVariableName", 'K(P)', MergeAsTable=true);
PaperTable = mergevars(PaperTable, {'Dice', 'Plank', 'Wedge'}, "NewVariableName", 'Rank', 'MergeAsTable', true);
PaperTable.Properties.RowNames = allMeasureSummaryTable.Properties.RowNames;

% PaperTable(:,"K(C)").Variables;
% PaperTable(:,"K(P)").Variables;


%% Convert paper table in to a coherent plot

GPC_plank.plotRank(PaperTable)
