%%Baseline Test
%   Code to generate random poses of the ant head and generate grasps that are
%   random but initiated by the ant moving towards the object with open
%   mandibles

function [shapeRange] = findBaselineShapeFunction(shapePath, shapeScale)

%bestMeasures, worstMeasures, medianMeasures

%% Define the object parameters and ant starting pose
RUNTIME_ARGS = RuntimeArgs();

RUNTIME_ARGS.COLLISION_OBJ.SCALE = shapeScale;
RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = shapePath;

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

nRandomPt = 200;

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

% %% Plot a subset to make sure they look right
figure;
title(shapePath, 'Interpreter','none')
subsetSize = 200;
subsetPForPlot = pointOnObj(1:subsetSize,:);
subsetFForPlot = normalVArray(1:subsetSize,:);

%Plotting
hold on
quiver3(subsetPForPlot(:,1),subsetPForPlot(:,2),subsetPForPlot(:,3), ...
    subsetFForPlot(:,1),subsetFForPlot(:,2),subsetFForPlot(:,3),'off','color','r');

%% For finding the worst (completely random) - find the quality for every pair then find mean

%[From the grasp synth class]

gE = ant.graspEval;


goalOut = goalStruct();
grasp_quality_array = repmat(graspQuality(), nRandomPt);

for n = 1:nRandomPt
    for m = 1:nRandomPt
        if n~=m
            contactStruct = struct();
            contactStruct(1).point = pointOnObj(n,:);
            contactStruct(2).point = pointOnObj(m,:);
            if any(contactStruct(1).point-contactStruct(2).point)
                contactStruct(1).normal = normalVArray(n,:);
                contactStruct(2).normal = normalVArray(m,:);
                goalOut_i = goalOut.setGraspContact(contactStruct);
                grasp_quality_array(n,m) = gE.evaluateGoal(goalOut_i, env);
            end

        end


    end
end
% Find which indices need to be removed
sz = size(grasp_quality_array);
d1 = 1:sz(1);
d2 = 1:sz(2);

volArray = cat(1,grasp_quality_array.volume);
epsArray = cat(1, grasp_quality_array.epsilon);
comoffArray = cat(1, grasp_quality_array.com_offset);
alignArray = cat(1,grasp_quality_array.normAlign);
reachArray = cat(1,grasp_quality_array.withinReach);
%% Remove self-pairs
emptyGrasp_idx = find(isinf(comoffArray));
volArray(emptyGrasp_idx) = [];
epsArray(emptyGrasp_idx) = [];
comoffArray(emptyGrasp_idx) = [];
alignArray(emptyGrasp_idx) = [];
reachArray(emptyGrasp_idx) = [];

%% Best overall grasp
% volumeScore = rescale(volArray);
% epsilonScore = rescale(epsArray);
% offsetScore = 1 - rescale(comoffArray);
% alignScore = rescale(alignArray);

%Sum/multiply all values to find which scores the best on all fronts
%Assumes all features are weighted equally
%Volume should be large, epsilon should be large, align should be large but
%offset should be small, so - offset
% 
% prodScore = volumeScore .* epsilonScore .* offsetScore .* alignScore .* reachArray;
% prodScore(prodScore == 0) = nan;

% [max_prod_score, prod_score_max_idx] = max(prodScore, [], 'all', "omitnan");
% bestGrasp = graspQuality();
% bestGrasp.volume = volArray(prod_score_max_idx);
% bestGrasp.epsilon = epsArray(prod_score_max_idx);
% bestGrasp.com_offset = comoffArray(prod_score_max_idx);
% bestGrasp.normAlign = alignArray(prod_score_max_idx);
% bestGrasp.withinReach = reachArray(prod_score_max_idx);


% [min_prod_score, prod_score_min_idx] = min(prodScore, [], "all", "omitnan");
% worstGrasp = graspQuality();
% worstGrasp.volume = volArray(prod_score_min_idx);
% worstGrasp.epsilon = epsArray(prod_score_min_idx);
% worstGrasp.com_offset = comoffArray(prod_score_min_idx);
% worstGrasp.normAlign = alignArray(prod_score_min_idx);
% worstGrasp.withinReach = reachArray(prod_score_min_idx);

%% Remove failed grasps
% failedGrasp_idx = isnan(prodScore);
% volArray(failedGrasp_idx) = [];
% epsArray(failedGrasp_idx) = [];
% comoffArray(failedGrasp_idx) = [];
% alignArray(failedGrasp_idx) = [];


shapeRange.volume.min = min(volArray,[], "all", "omitnan");
shapeRange.volume.max = max(volArray,[], "all", "omitnan");
shapeRange.epsilon.min = min(epsArray,[], "all", "omitnan");
shapeRange.epsilon.max = max(epsArray,[], "all", "omitnan");
shapeRange.com_offset.min = min(comoffArray,[], "all", "omitnan");
shapeRange.com_offset.max = max(comoffArray,[], "all", "omitnan");
shapeRange.normAlign.min = min(alignArray,[], "all", "omitnan"); 
shapeRange.normAlign.max = max(alignArray,[], "all", "omitnan");

shapeRange.volArray = volArray;
shapeRange.epsArray = epsArray;
shapeRange.comoffArray = comoffArray;
shapeRange.alignArray = alignArray;
shapeRange.reachArray = reachArray;


end
