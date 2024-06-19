%% File to plot the new GPC when force has been changed
% % Created 28/02/2023
% Saved as GPC_full_force1.mat

%% Plot to show how the knee points are used to select percentage
experimentNameExample = "IPDAlign_RRaP";
GPC_wedge.percent_colour = [1 0.55 0];
GPC_wedge.plotKneeSelection(experimentNameExample)

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
percentPass = 0.3;
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

%% Order by map table not by rank
allMeasureSummaryTable = GPC_wedge.reorderRows(allMeasureSummaryTable, ["Control Method", "Search Method", "Row Index"]);


%% Copy out the max percent and knee in order of the measures summary Table

PaperTable = array2table([completeDiceTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee", "SimulationTime MaxKnee", "RealWorldTime MaxKnee"]).Variables, ...
            completePlankTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee", "SimulationTime MaxKnee", "RealWorldTime MaxKnee"]).Variables, ...
            completeWedgeTable(allMeasureSummaryTable.Properties.RowNames, ["KneeNContact Max", "PercentSuccess MaxKnee", "SimulationTime MaxKnee", "RealWorldTime MaxKnee"]).Variables, ...
           allMeasureSummaryTable{:,["Max Max Shape Score","Max Mean Score", "SD Max Shape Score"]}]);
PaperTable.Properties.VariableNames = {'Dice K(C)', 'Dice K(P)', 'Dice tau_S', 'Dice tau_RW', 'Plank K(C)', 'Plank K(P)', 'Plank tau_S', 'Plank tau_RW', 'Wedge K(C)', 'Wedge K(P)', 'Wedge tau_S', 'Wedge tau_RW', 'Worst Score','Worst Mean Score', 'SD of Worst Shape Score'};

PaperTable = mergevars(PaperTable, {'Dice K(C)', 'Plank K(C)', 'Wedge K(C)'}, "NewVariableName", 'K(C)', 'MergeAsTable',true);
PaperTable = mergevars(PaperTable, {'Dice K(P)', 'Plank K(P)', 'Wedge K(P)'}, "NewVariableName", 'K(P)', MergeAsTable=true);
PaperTable = mergevars(PaperTable, {'Dice tau_S', 'Plank tau_S', 'Wedge tau_S'}, "NewVariableName", 'tau S', MergeAsTable=true);
PaperTable = mergevars(PaperTable, {'Dice tau_RW', 'Plank tau_RW', 'Wedge tau_RW'}, "NewVariableName", 'tau RW', MergeAsTable=true);
%PaperTable = mergevars(PaperTable, {'Dice', 'Plank', 'Wedge'}, "NewVariableName", 'Rank', 'MergeAsTable', true);
PaperTable.Properties.RowNames = allMeasureSummaryTable.Properties.RowNames;

% PaperTable(:,"K(C)").Variables;
% PaperTable(:,"K(P)").Variables;


%% Convert paper table in to a coherent plot

GPC_plank.plotPaperRank(PaperTable)

%%
GPC_plank.plotPaperKnee(PaperTable)
