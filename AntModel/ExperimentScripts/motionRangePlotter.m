%% From a record of motion, and the rigid body tree, plot the volume of motion created by the key points on the antennae

close all;
clear;

modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\';
%Save the current script location
scriptFolder = pwd;
cd(modelFolder)

%Add the subfolders within AntModel to the path 
addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
    'urdf', 'Grasp', 'ExperimentOutput', 'ExperimentScripts', 'MainScript', 'ToolClasses');

experimentFolder = 'ExperimentOutput\contactTimeTestScript\';
trialFolder = '_04-04-23_16-38\';
matFolder = 'mat-files\trial_1.mat';

load([modelFolder,experimentFolder,trialFolder,matFolder])

%% Find the 3D pose of the antennal joints for all poses in the replay table
nStep = size(replayTable,1);
globalPosition = nan(nStep,6,3);
endEffectorName = {'right_antenna_base'; 'r_scape';	'r_tip'; 'left_antenna_base'; 'l_scape'; 'l_tip'};

for r = 1:nStep
    qIn = replayTable.Pose{r};
    positionIn = replayTable.Position(r,:);
    for n = 1:length(endEffectorName)
        globalPosition(r,n,:) = tbox.findFKglobalPosition(antTree, qIn, positionIn, endEffectorName{n});
    end

end


%% Compile all the 3D points in to a convex hull

allTrialPosition = reshape(globalPosition,[],3);

h = convhull(allTrialPosition, 'Simplify', true)
trisurf(h,allTrialPosition(:,1),allTrialPosition(:,2),allTrialPosition(:,3),'FaceColor','cyan')
axis equal

