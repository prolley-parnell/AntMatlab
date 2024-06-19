%% File to test the functions of the GraphPlotClass
% Created 23/01/2023

%GPC = GraphPlotClass();
% 
% %Add all Experiment Folders
%experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
%GPC = GPC.loadData;
%save('GPC_Example.mat', 'GPC')


%% Writing Exclude Failed Grasps Code
experimentName = ["IPDAlign_RSS"];
measureName = ["Volume", "COM Offset","Epsilon", "Align"];
refineFlag = 0;
[~, skewVal] = GPC.experimentPDF(experimentName, measureName, refineFlag);

%% Refine the data (testing) Transform and Find Median
experimentName = ["IPDAlign_RSS"];
measureName = [GPC.all_quality_name, "Simulation Time", "Real World Time"];
[~, yData] = GPC.extractMeasure(experimentName, measureName);
[refinedDataOut, successNumber, failureNumber] = GPC.excludeFailedGrasp(yData{1});
medianVal = GPC.transformMedian(refinedDataOut, measureName);
%% Find the knee for the median values across the first row
kneeX = GPC.findKnee(xData{1}, medianVal(1,:));

%% Finding Cross Correlation
allExperimentName = {GPC.experimentDataStruct.Title};
allQualityName = ["Volume", "Epsilon", "Align", "COM Offset"];
[xData, yData] = GPC.extractMeasure(allExperimentName, allQualityName)

%Compound all the measures
goalSample = reshape(yData{:}, [], 4);
[R, P] = corrcoef(goalSample,'Rows','complete')


%% Plot the full table of behaviours against the various measures
%[~, mapTable, resultCellArray] = GPC.completePaperPlot
[resultTable] = GPC.extractPlotAndCostData;
GPC.plotResults(resultTable);

%% Rank each behaviour to find the best
[resultTable] = GPC.extractPlotAndCostData;
maxKneeTable = GPC.findMaxKnee(resultTable);
%% 
pass05Table = GPC.addPercentPassColumn(maxKneeTable, 0.5);

[pass05TableRank] = GPC.addRank(pass05Table);

EndTable05 = GPC.addSummaryRank(pass05TableRank);

%allMeasureSummaryTable = GPC.summariseTableRank(EndTable05, EndTable06);


%%
%[orderedResultTable] = GPC.orderGraspQualityPlateau(resultTable);

%%
[resultTable] = GPC.extractPlotAndCostData;
[orderedResultTable] = GPC.orderGraspQualityPlateau(resultTable);
GPC.plotOrderedResultTable(orderedResultTable)
%%
GPC.plotOrderedMedianTable(orderedResultTable)

%%
GPC.plotOrderedMedianQualityAndCost(orderedResultTable)
