classdef goalStruct
    %goalStruct Properties and functions required for a goalStruct


    properties (SetAccess = public, GetAccess = public)
        qualityObj graspQuality

        time_s = -1
        midpoint = nan([1,3],"double")
        contact_axis = nan([1,3],"double")
        contact_point_array = nan([2,3],"double")
        alignment_axis = nan([1,3],"double")
        goal_z_axis = nan([1,3],"double")
        contact_norm = nan([2,3], "double")


    end
    properties (SetAccess = private, GetAccess = protected)
        emptyFlag = true
    end


    methods
        function obj = goalStruct(~)
            %goalStruct constructor, creates an empty instance of the class
            %quality = A class that can be used and have multiple measures of quality attached

        end

        function [obj] = setGraspContact(obj, contactStructArray)
            %SETCONTACT given the two contact points that make up the
            %attempted grasp (not the ant-desire grasp), extract the key
            %features for evaluation
            % Input: 
            % contactStructArray: struct containing the point, and normal
            % of the grasped contacts
            % Output:
            % obj: updated goalStruct instance with the final grasp
            % properties
            
            
            pointArray = cat(1,contactStructArray(:).point);
            normArray = cat(1,contactStructArray(:).normal);
            obj.emptyFlag = false;
            obj.contact_point_array = pointArray;
            obj.contact_axis = pointArray(1,:) - pointArray(2,:);
            obj.midpoint = mean(pointArray, 1);
            obj.contact_norm = normArray;
            
        end

        function flag = isempty(obj)
            flag = obj.emptyFlag;
        end

        function outTable = convert2table(obj)
            time = obj.time_s;
            mp = obj.midpoint;
            axis = obj.contact_axis;
            a = obj.contact_point_array(1,:);
            b = obj.contact_point_array(2,:);
            norm_a = obj.contact_norm(1,:);
            norm_b = obj.contact_norm(2,:);

            goalCell = [{time, mp, axis, a, norm_a, b, norm_b}];
            goalTable = cell2table(goalCell);
            goalTable.Properties.VariableNames = {'Time', 'Midpoint', 'Contact Axis', 'Point A', 'Surf Norm A', 'Point B', 'Surf Norm B'};

            if ~isempty(obj.qualityObj)
                qualityTable = obj.qualityObj.convert2table();
                % append the quality to the table
            else
                qualityTable = [];
            end
            outTable = [goalTable , qualityTable];
        end

        function obj = fromTable(obj, tableIn)
                        
            obj.emptyFlag = false;
            obj.time_s = tableIn.Time;
            obj.midpoint = tableIn.Midpoint;
            obj.contact_point_array = [tableIn.("Point A");tableIn.("Point B")];
            obj.contact_axis = tableIn.("Contact Axis");
            obj.contact_norm = [tableIn.("Surf Norm A");tableIn.("Surf Norm B")];

        end

        function obj = settime(obj, currentTime)
            obj.time_s = currentTime;
        end

        function [obj] = saveGoalApproach(obj, axesIn)
            %SETGOALAPPROACH Set the axes used to plot the end approach
            % Input:
            % axesIn: struct with X,Y,Z and MP, all vectors point outwards
            % from the MP
            % Output:
            % obj: goalStruct object now with alignment an goal axes set

            
            obj.alignment_axis = -axesIn.Y;
            obj.goal_z_axis = axesIn.Z;

           
        end
        
        function obj = plotGoal(obj, PLOT)
            if any(PLOT.ENABLE)
                vector_base = obj.midpoint - obj.alignment_axis;
                nPlot = length(PLOT.ENABLE);
                for i = 1:nPlot
                    if PLOT.ENABLE(i)
                        figure(i)
                        hold on;
                        plot3(obj.contact_point_array(:,1), obj.contact_point_array(:,2), obj.contact_point_array(:,3), ':*','MarkerSize',7, 'LineWidth', 1)
                        quiver3(obj.midpoint(1), obj.midpoint(2), obj.midpoint(3), obj.contact_axis(1), obj.contact_axis(2), obj.contact_axis(3), 'LineWidth', 3, 'Color', 'r')
                        quiver3(vector_base(1), vector_base(2), vector_base(3), obj.alignment_axis(1), obj.alignment_axis(2), obj.alignment_axis(3), 'LineWidth', 3, 'Color', 'g')
                        quiver3(obj.midpoint(1), obj.midpoint(2), obj.midpoint(3), obj.goal_z_axis(1), obj.goal_z_axis(2), obj.goal_z_axis(3), 'LineWidth', 3, 'Color', 'b')
                        hold off;
                    end
                end
            end
        end

    end
end