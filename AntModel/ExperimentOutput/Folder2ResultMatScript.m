%Find the average goal stability for every experiment, and compare against
%test criterion
%For each subfolder, sum the values for each excel trial
clear all;
%Import the folder name to process

folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction\refineIGEFAlignDiceObj2-40';
folderStruct = dir(folder_name);

resultArray = [];
experimentIndex = 1;
%For all folders that are not hidden (experiments)
for i = 1:length(folderStruct)
    if ~strcmp(folderStruct(i).name(1), '.') && isfolder([folder_name, '\', folderStruct(i).name])
        %"printout" is the subfolder name for xls files in the runtime args
        subfolder = ['\', folderStruct(i).name, '\printout'];
        subDir = [folder_name, subfolder];


        experimentStruct = dir([subDir, '\*.xls']);
        experimentTable = table();

        for j = 1:length(experimentStruct)
            fileName = [subDir, '\', experimentStruct(j).name];
            sheetNamesj = sheetnames(fileName);
            for k = 1:length(sheetNamesj)
                if strcmp(sheetNamesj(k), 'Sensed Goal')
                    trialTable = readtable(fileName, 'FileType','spreadsheet', 'Sheet', k);
                    experimentTable(j,:) = trialTable;
                end
            end


        end

        experimentQuality = [experimentTable.Volume(:), experimentTable.Epsilon(:), experimentTable.COMOffset(:), experimentTable.normAlign(:)];
        name_split = split(folderStruct(i).name,'_');
        nContactArray = repmat(str2double(name_split{1}),[length(experimentStruct),1]);
        resultArray = cat(1, resultArray, [nContactArray,experimentQuality]);


        experimentIndex = experimentIndex + 1;

    end


end

%%

save([folder_name, '\allQualities.mat'], 'resultArray');


%%
statTable = table();

[~, sortIndx] = sort(resultArray(:,1));
sortedResultArray = resultArray(sortIndx,:);

nContact = unique(sortedResultArray(:,1));

for k = 1:length(nContact)

    expSortIndx = find(sortedResultArray(:,1) == nContact(k));
    experimentQualityAll = sortedResultArray(expSortIndx,:);

    statTable.meanVolume(k) = mean(experimentQualityAll(:,2));
    statTable.varVolume(k) = var(experimentQualityAll(:,2));
    statTable.minmaxVolume(k,:) = [min(experimentQualityAll(:,2)) , max(experimentQualityAll(:,2))];

    statTable.meanEpsilon(k) = mean(experimentQualityAll(:,3));
    statTable.varEpsilon(k) = var(experimentQualityAll(:,3));
    statTable.minmaxEpsilon(k,:) = [min(experimentQualityAll(:,3)) , max(experimentQualityAll(:,3))];

    statTable.meanCOMOffset(k) = mean(experimentQualityAll(:,4));
    statTable.varCOMOffset(k) = var(experimentQualityAll(:,4));
    statTable.minmaxOffset(k,:) = [min(experimentQualityAll(:,4)) , max(experimentQualityAll(:,4))];

    statTable.meanNormAlign(k) = mean(experimentQualityAll(:,5));
    statTable.varNormAlign(k) = var(experimentQualityAll(:,5));
    statTable.minmaxNormAlign(k,:) = [min(experimentQualityAll(:,5)) , max(experimentQualityAll(:,5))];

    statTable.Properties.RowNames{k} = int2str(nContact(k));
    
end

save([folder_name, '\experimentStatTable.mat'], 'statTable');