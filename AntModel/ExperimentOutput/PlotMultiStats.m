%Plot multiple results files on the same graph

%Plot the results from the resultsTable file

close all;
clear all;


% folder_name{1} = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\DistAndAlignDiceObj2-40';
% folder_name{2} = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\DistOnlyDiceObj2-40';
% folder_name{3} = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\AlignOnlyDiceObj2-40';
folder_name{1} = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\refineIGEFAlignDiceObj2-40';
folder_name{2} = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\TunedIGEFAlignDiceObj2-40';


nFolder = length(folder_name);

file_name = 'experimentStatTable.mat';

variableNumber = 4;
names = {'Grasp Wrench Volume', 'Epsilon', 'Centre Of Mass Offset', 'Force Alignment'};


meanTableCell = cell([1,nFolder]);
varTableCell = cell([1,nFolder]);
numberOfContactCell = cell([1,nFolder]);
experimentLegend = {'Untuned IGEF', 'Tuned IGEF'};

for m=1:nFolder

    matStruct = load([folder_name{m}, '\', file_name]);
    statTable = matStruct.statTable;

    %experimentName = split(folder_name{m}, '\');
    %experimentLegend{m} = experimentName{end};

    for i=1:length(statTable.Properties.RowNames)
        name_split = split(statTable.Properties.RowNames(i),'_');
        numberOfContacts(i,:) = str2num(name_split{1});
    end



    meanT = [statTable.meanVolume, statTable.meanEpsilon, statTable.meanCOMOffset, statTable.meanNormAlign];
    varT = [statTable.varVolume, statTable.varEpsilon, statTable.varCOMOffset, statTable.varNormAlign];
    % minT = [statTable.minmaxVolume(:,1), statTable.minmaxEpsilon(:,1), statTable.minmaxOffset(:,1), statTable.minmaxNormAlign(:,1)];
    % maxT = [statTable.minmaxVolume(:,2), statTable.minmaxEpsilon(:,2), statTable.minmaxOffset(:,2), statTable.minmaxNormAlign(:,2)];

    [NoC_ordered, i_ordered] = sort(numberOfContacts);

    numberOfContactCell{m} = NoC_ordered;
    meanTableCell{m} = meanT(i_ordered,:);
    varTableCell{m} = varT(i_ordered,:);
    % varT = varT(i_ordered,:);
    % minT = minT(i_ordered,:);
    % maxT = maxT(i_ordered,:);



end

% newcolors = [0.83 0.14 0.14
%     1.00 0.54 0.00
%     0.47 0.25 0.80
%     0.25 0.80 0.54];

newcolors = [1.00 0.54 0.00
    0.47 0.25 0.80
    0.25 0.80 0.54];

colororder(newcolors)
styleCell = {'-', '--', '-.'};

fg = tiledlayout(variableNumber,1)
nContact = nan([1,nFolder]);
for l = 1:nFolder
    nContact(l) =  length(numberOfContactCell{l});
end
maxNContact = min(nContact);

for i = 1:variableNumber
    nexttile
    for j = 1:nFolder

        hold on
        title(names{i}, 'FontAngle', 'italic', 'FontWeight','normal')


        meanExpData = meanTableCell{j};

        expContact = numberOfContactCell{j};
        plot(expContact(1:maxNContact), meanExpData(1:maxNContact,i),styleCell{j}, 'DisplayName', experimentLegend{j});

        %             yyaxis right
        %             ylabel("Variance")
        %             varExpData = varTableCell{j};
        %
        %             plot(expContact,varExpData(:,i), ':');


    end

    hold off

end

hold on
title(fg, {'IGEF Tuned vs Untuned for Alignment Grasps'}, 'FontWeight', 'Bold')
xlabel(fg, "Number of Contacts");
ylabel(fg, "Mean Value")
hold off






