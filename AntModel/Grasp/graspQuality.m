classdef graspQuality
    %% graspQuality class that contains all the different measures of quality that may be used to evaluate a proposed grasp position
    %%
    properties
        epsilon (1,1) double = 0 %The radius of a 6D sphere about the object centre of mass
        %that is contained within the Convex Wrench Hull of the grasp
        com_offset (1,1) double = inf %Distance between the mean of all the grasp contact points
        %and the true centre of mass of the object
        volume(1,1) double = 0 %Volume of the Grasp Wrench Space represented as a convex hull
        % of the force approximation vectors that result from 
        normAlign(1,1) double = 0 %Scalar between -1 and 1 to indicate how much of the surface 
        % normal at the points of contact are aligned with the force vectors
        withinReach(1,1) double = 1 %Boolean flag to indicate whether the grasp selected is within the reach of the mandibles
    end

    methods
        function obj = graspQuality(~)
            %% graspQuality Construct an empty instance of this class
        end

        function qualityTable = convert2table(obj)
            %% CONVERT2TABLE Produce a table object containing the properties and labels associated with the graspQuality class
            qualityCell = {obj.volume, obj.epsilon, obj.com_offset, obj.normAlign, obj.withinReach};
            qualityTable = cell2table(qualityCell);
            qualityTable.Properties.VariableNames = {'Volume', 'Epsilon', 'COM Offset', 'normAlign', 'withinReach'};
        end
    end
end