%%Baseline Test
%   Code to generate random poses of the ant head and generate grasps that are
%   random but initiated by the ant moving towards the object with open
%   mandibles

%% Add folders to path
close all;
clear;
modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\';
scriptFolder = pwd;

cd(modelFolder)

addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
    'urdf', 'Grasp', 'ExperimentOutput', 'MainScript', 'ToolClasses');

rng('shuffle');

%% Define the object parameters and ant starting pose
RUNTIME_ARGS = RuntimeArgs();

RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.18;
RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = './Environment/12_sided_tiny_shape.stl';
RUNTIME_ARGS.COLLISION_OBJ.POSITION = [0 3 0.5];

RUNTIME_ARGS.ANT_POSE = [0 0 0 0 0.3 -0.45 0.85 0.3 -0.45 0.85]';
RUNTIME_ARGS.ANT_POSITION = [0 0 0 0];

%% Load in the model and object
RUNTIME_ARGS.PLOT.ENABLE = [1 0];
RUNTIME_ARGS.initiatePlots();

%Define Ant object
ant = Ant(RUNTIME_ARGS);

%Define the collision objects in the model environment
env = CollisionObjects(RUNTIME_ARGS);


%% Generate a large number of points in space
%Find the range of the vertices that make up the shape to refine the search
%space for point generation
%Currently for single shape [TODO]
vert = env.FBT{1}.Points;
%Find maximum  and minimum vertices
maxV = max(vert);
minV = min(vert);

nRandomPt = 1000;

r = @(a, b, set) (a + (b-a).*set);
randomScale = rand([nRandomPt,3]);

randomSampleFull = r(minV,maxV,randomScale);

%% Find the points on the surface of the object that are guided by these points
% for single triangulation only 
%[TODO] need to remove points that are
% considered to be internal
%{TODO] Can I do this by literally adding all of the triangulations
%together?

%For each point, find the face it is contained by
[pointOnObj, normalVArray, vertIDArray, faceIDArray] = tbox.findPointOnObjNormalID(env.FBT{1}, randomSampleFull);

%% Plot a subset to make sure they look right
% subsetSize = 5000;
% subsetPForPlot = pointOnObj(1:subsetSize,:);
% subsetFForPlot = normalVArray(1:subsetSize,:);
% 
% %Plotting
% hold on
% quiver3(subsetPForPlot(:,1),subsetPForPlot(:,2),subsetPForPlot(:,3), ...
%     subsetFForPlot(:,1),subsetFForPlot(:,2),subsetFForPlot(:,3),'off','color','r');

%% For finding the worst (completely random) - find the quality for every pair then find mean

%[From the grasp synth class]

obj = ant.graspGen;

contactStruct = struct();
contactStruct(:).point = pointOnObj;
contactStruct(:).normal = normalVArray;

goalOut = goalStruct();

%Cartesian contact points
cartPointArray = pointOnObj;
nContactPoint = size(cartPointArray,1);

[distanceMat, ~, ~] = obj.findInterPointDistance(cartPointArray, "none");

mandMaxFlag = obj.mandibleLimit(distanceMat);

[alignMeasure, ~, ~] = obj.findInterPointGraspAlign(contactStruct);

qualityMap = distanceMat.*mandMaxFlag.*alignMeasure;
[bestQuality, goalIndex] = max(qualityMap,[],'all');

%% Find the mean interpoint distance
meanIPD = mean(distanceMat,'all', 'omitnan');
%% Save
save("comparisonQualities.mat", "distanceMat", "mandMaxFlag", "alignMeasure", "contactStruct");

%% Find the mean midpoint offset

%Find midpoint of all pairs
COMOffset = nan([nContactPoint, nContactPoint]);
idx = 1;
objPose = env.objectHandles{idx}.Pose;
objCOM = env.COM{idx};
globalCOM = tbox.local2global(objCOM, objPose);
for n = 1:nContactPoint
    for m = n:nContactPoint
        if n~=m
            mp = mean([contactStruct.point(n,:); contactStruct.point(m,:)]);
            COMOffset(n, m) = vecnorm(mp-globalCOM,2,2);

        end
    end
end
meanCOMOffset = mean(COMOffset,'all', 'omitnan');
%%
save("comparisonQualities.mat", "distanceMat", "mandMaxFlag", "alignMeasure", "contactStruct", "COMOffset");

%% Find the mean wrench quality


%Calculate the volume and epsilon for the given goal grasp
%point locations
obj = ant.graspEval;
vertices = env.objectHandles{idx}.Vertices;
volumeArray = nan([nContactPoint, nContactPoint]);
epsilonArray = nan([nContactPoint, nContactPoint]);


for n = 1:nContactPoint
    for m = n:nContactPoint
        if n~=m
            A = contactStruct.point(n,:);
            B = contactStruct.point(m,:);
            if any([A-B])
                forces = obj.genOpposeForces(A, B);
                [volumeArray(n,m), epsilonArray(n,m)] = obj.findWrenchQuality(forces, [A;B], vertices, globalCOM, objCOM);
            else
                volumeArray(n,m) = 0;
                epsilonArray(n,m) = 0;
            end

        end
    end
end



%% Load the file if necessary

load('C:\Users\eroll\Documents\MATLAB\Model\Debugging Tools\comparisonBaselineResults.mat')




%% For finding best (perfect placement) - find the maximum quality grasps

%Convert all quality measures to a scalar between lowest and highest (worst
%and best)
volumeScore = rescale(volumeArray);
epsilonScore = rescale(epsilonArray);
offsetScore = rescale(COMOffset);
alignScore = rescale(alignMeasure);

%Sum/multiply all values to find which scores the best on all fronts
%Assumes all features are weighted equally
%Volume should be large, epsilon should be large, align should be large but
%offset should be small, so - offset
sumScore = (volumeScore + epsilonScore - offsetScore + alignScore) .* mandMaxFlag;
% prodScore = volumeScore .* epsilonScore .* offsetScore .* alignScore .* mandMaxFlag;

[max_sum_score, sum_score_max_idx] = max(sumScore, [], 'all');
[contact_a_idx_sum, contact_b_idx_sum] = ind2sub(size(sumScore), sum_score_max_idx);

%Find the values that associate with the max
sumScoreMat = cat(3, volumeScore, epsilonScore, offsetScore, alignScore);
bestValue = squeeze(sumScoreMat(contact_a_idx_sum, contact_b_idx_sum,:));


% [max_prod_score, prod_score_max_idx] = max(prodScore, [], 'all');
% [contact_a_idx_prod, contact_b_idx_prod] = ind2sub(size(prodScore), prod_score_max_idx);

%% Summarise the qualities of this experiment

%Show box plots for each variable

dataCells = {volumeScore, epsilonScore, offsetScore, alignScore};
qualityNames = {'Volume', 'Epsilon', 'COM Offset', 'Alignment'};
nMeasure = length(dataCells);
compressed_data = nan([nRandomPt^2 , nMeasure]);

for measure_i = 1:nMeasure
    compressed_qual = reshape(dataCells{measure_i}, 1, []);
    compressed_data(:,measure_i) = compressed_qual;


end


%% Plotting PDF for one variable
pd = fitdist(compressed_data(:,1), 'Normal');
x_values = [0:0.001:0.2]; %Width decided by observation
y = pdf(pd, x_values);
plot(x_values, y, 'LineWidth', 2)
%%
boxplot(compressed_data, qualityNames', 'BoxStyle', 'filled');
xlabel("Quality Measures for Dice scale 0.18")
ylabel("Normalised Quality Score")
hold on
plot(bestValue,'-o')
hold off
legend(["Best Grasp Qualities"])


%% What if I try PCA on the data?

%Use the original data
dataCells = {volumeArray, epsilonArray, COMOffset, alignMeasure};
qualityNames = {'Volume', 'Epsilon', 'COM Offset', 'Alignment'};
nMeasure = length(dataCells);
X = nan([nRandomPt^2 , nMeasure]);

for measure_i = 1:nMeasure
    compressed_qual = reshape(dataCells{measure_i}, 1, []);
    X(:,measure_i) = compressed_qual;
end

% Remove any rows where NaN makes the covariance impossible
X(any(isnan(X),2),:) = [];
CM = cov(X);
% Step 2:  Eigenvector and Eigenvalue
[V, D]= eig(CM);

% Step 3: Sort the Eigenvectors according to eigenvalue
eVal = diag(D);
[decend_eVal, idx_eVec] = sort(eVal, 1, "descend");
decend_eVec = V(:,[idx_eVec]);


% Step 4: Store Eigenvectors in Projection Matrix
% k desired features/dimesion reduction
k = 3;
W = decend_eVec(:,[1:k]);

% Step 5: Transform original data in to the Principal Component
Y = X*W;

%% Plot the PC box plots
figure(1)
boxplot(X, qualityNames', 'BoxStyle', 'filled');
xlabel("Quality Measures for Dice scale 0.18")
ylabel("Normalised Quality Score")
title("Trimmed Data")
hold on
plot(bestValue,'-o')
hold off
legend(["Best Grasp Qualities"])

%%
[~, startIdx] = min(Y);
P = X(startIdx,:);
Wi = 2*( W'.* decend_eVal(1:k));

%%
figure(2)
scatter3(Y(:,1),Y(:,2),Y(:,3), 0.1, 'ColorVariable', sqrt(rescale(Y(:,3))))

hold on
quiver3(P(:,1),P(:,2), P(:,3), Wi(:,1), Wi(:,2), Wi(:,3), 'off')



%%


% Identify the qualities of the best all round grasp



% Identify the maximum and minimum possible values for this shape


%% Find the grasps the score the best on each single measure and see how the
%other qualities compare to the best all round grasp



