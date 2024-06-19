%Plot the results from the resultsTable file

close all;
clear all;


folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\DistAndAlignDiceObj2-40';
file_name = 'experimentStatTable.mat';

matStruct = load([folder_name, '\', file_name]);
statTable = matStruct.statTable;

for i=1:length(statTable.Properties.RowNames)
    name_split = split(statTable.Properties.RowNames(i),'_');
    numberOfContacts(i,:) = str2num(name_split{1});
end



meanT = [statTable.meanVolume, statTable.meanEpsilon, statTable.meanCOMOffset, statTable.meanNormAlign];
varT = [statTable.varVolume, statTable.varEpsilon, statTable.varCOMOffset, statTable.varNormAlign];
minT = [statTable.minmaxVolume(:,1), statTable.minmaxEpsilon(:,1), statTable.minmaxOffset(:,1), statTable.minmaxNormAlign(:,1)];
maxT = [statTable.minmaxVolume(:,2), statTable.minmaxEpsilon(:,2), statTable.minmaxOffset(:,2), statTable.minmaxNormAlign(:,2)];

[NoC_ordered, i_ordered] = sort(numberOfContacts);

numberOfContacts = NoC_ordered;
meanT = meanT(i_ordered,:);
varT = varT(i_ordered,:);
minT = minT(i_ordered,:);
maxT = maxT(i_ordered,:);

variableNumber = 4;
names = {'Volume', 'Epsilon', 'COMOffset', 'Surface Alignment'};

%Remove outlier for no refine dice at 5 contacts


tiledlayout(variableNumber,1)
for j = 1:variableNumber
    nexttile
    hold on
    title(names{j})
    
    yyaxis left
    ylabel("Mean")
    %errorbar(numberOfContacts, meanT(:,j), minT(:,j), maxT(:,j))
    
    plot(numberOfContacts, meanT(:,j));

    yyaxis right
    ylabel("Variance")
    varplot = [numberOfContacts, varT(:,j)];

    plot(varplot(:,1),varplot(:,2));
    hold off
end
hold on
    xlabel("Number of Contacts");
    hold off


