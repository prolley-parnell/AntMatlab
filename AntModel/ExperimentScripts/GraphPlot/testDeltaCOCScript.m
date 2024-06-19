% SCRIPT to Extract the Delta COC in experiments

%GPC_dcoc = GraphPlotClass();
% experimentFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
% GPC_dcoc = GPC_dcoc.loadData('variables', {'contactsTable'}, [experimentFolder, '\dice']);

% Plot delta COM for one trial
% experimentDataStruct(1) = 'Align_CGMMC_fixvar' ( Location, Mean centred)
%%
E = 4;
dataSummary = struct();
for contacts_number = 1:length(GPC_dcoc.experimentDataStruct(1).trialData)
    % Find the grasp quality for each of these trials (Only copied out number 1 for now because of difficulty in copying them across en masse)
    GPC_dice.experimentDataStruct(E).trialData(contacts_number).contactsTable = GPC_dcoc.experimentDataStruct(E).trialData(contacts_number).contactsTable;
    GPC_dcoc.experimentDataStruct(E).trialData(contacts_number) = GPC_dice.experimentDataStruct(E).trialData(contacts_number);



    %From the GPC, make a copy of the relevant information in the contacts
    %table
    reduceContTable = GPC_dcoc.experimentDataStruct(E).trialData(contacts_number).contactsTable(:,["Trial Number", "Time", "Contact Location"]);
    %Find a list of the Trial ID values (1 to 50)
    trialID = unique(reduceContTable.("Trial Number"));
    %Make an empty struct to hold the experiment information


    %For all trials
    for i = 1:length(trialID)
        %Make a subtable containing all contacts from a single trial
        trialData = reduceContTable(reduceContTable.("Trial Number") == trialID(i), ["Time", "Contact Location"]);
        %Create inf arrays to contain the COC and delta COC
        COC = inf(height(trialData), 3);
        delta = inf(height(trialData), 1);

        %For each contact
        for j = 1:height(trialData)
            %Calculate the mean of all the contact points up to that point
            COC(j,:) = mean(trialData{1:j,"Contact Location"}, 1);
            %If the first COC has been calculated, also calculate delta COC
            if j>=2
                delta(j,:) = vecnorm(COC(j-1,:)-COC(j,:), 2, 2);
            end
        end
        % Assign all COC for the trial to dataSummary i
        dataSummary(contacts_number,i).COC = COC;

        %Assign any remaining inf values to be nan instead
        delta(isinf(delta)) = nan;
        %Save the delta from this trial
        dataSummary(contacts_number,i).delta = delta;

        % Set up the x axis labels for the number of contacts
        xData = [1:length(COC)];

        %Find the difference betweent the COC and the true mean
        dataSummary(contacts_number,i).COC_Offset = vecnorm(COC - [0 3 0.5], 2, 2);

        dataSummary(contacts_number,i).align = GPC_dcoc.experimentDataStruct(E).trialData(contacts_number).senseGoalTable.Align(i);
        dataSummary(contacts_number,i).volume = GPC_dcoc.experimentDataStruct(E).trialData(contacts_number).senseGoalTable.Volume(i);
        dataSummary(contacts_number,i).COM_grasp_offset = GPC_dcoc.experimentDataStruct(E).trialData(contacts_number).senseGoalTable.("COM Offset")(i);
        dataSummary(contacts_number,i).COC_Offset_end = dataSummary(contacts_number,i).COC_Offset(end);

    end

end
%% Does the end COC from the true COM result in a grasp with a larger volume?

x = cat(2,dataSummary.COC_Offset_end);
y = cat(2,dataSummary.COM_grasp_offset);
z = cat(2,dataSummary.align);

hold on
title(GPC_dcoc.experimentDataStruct(E).Title, 'Interpreter','none')
xlabel("COC offset from true COM")
ylabel("Grasp MP offset from COM")
zlabel("Grasp Align measure")

scatter3(x,y,z)



% Plot PCA axes
combine_array = [x', y', z'];
combine_array(any(isnan(combine_array), 2), :) =[];
% Step 1: Calculate the covariance matrix
CM = cov(combine_array);
% Step 2:  Eigenvector and Eigenvalue
[V, D]= eig(CM);

% Step 3: Sort the Eigenvectors according to eigenvalue
eVal = diag(D);
[~, idx_eVec] = sort(eVal, 1, "descend");
decend_eVec = V(:,[idx_eVec]);

% Step 4: Store Eigenvectors in Projection Matrix
% k desired features/dimension reduction = 1 to find largest
% eigenvalue
var_axis = decend_eVec(:,1)';


mean_pca = mean(combine_array, 1);

quiver3(mean_pca(1), mean_pca(2), mean_pca(3), var_axis(1), var_axis(2), var_axis(3), 'off')




%%

% experimentDataStruct(1) = 'Align_CGMMC_fixvar' ( Location, Mean centred)
T = tiledlayout(4,4);
for E = 1:16
    nexttile
    hold on
    title(GPC_dcoc.experimentDataStruct(E).Title, 'Interpreter','none')
    reduceContTable = GPC_dcoc.experimentDataStruct(E).trialData(33).contactsTable(:,["Trial Number", "Time", "Contact Location"]);
    trialID = unique(reduceContTable.("Trial Number"));
    dataSummary = struct();
    for i = 1:length(trialID)
        trialData = reduceContTable(reduceContTable.("Trial Number") == trialID(i), ["Time", "Contact Location"]);
        COC = inf(height(trialData), 3);
        delta = inf(height(trialData), 1);
        for j = 1:height(trialData)
            COC(j,:) = mean(trialData{1:j,"Contact Location"}, 1);
            if j>=2
                delta(j,:) = vecnorm(COC(j-1,:)-COC(j,:), 2, 2);
            end
        end
        dataSummary(i).COC = COC;
        delta(isinf(delta)) = nan;
        dataSummary(i).delta = delta;
        xData = [2:length(COC)+1];
        xData(isnan(delta))=[];
        delta(isnan(delta))=[];
        dataSummary(i).xKnee = GPC_dcoc.findKnee(xData, delta);
    end

    histfit(cat(1, dataSummary(:).xKnee))
    hold off

end
A = dataSummary(22).delta;
[LT,ST,R] = trenddecomp(A)