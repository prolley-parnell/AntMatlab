classdef CostCalculator
    %CostCalculator contains the functions and values used to evaluate the
    %costs incurred by the model in a single trial. Specifically for the
    %evaluation at the end, not for internal use.

    properties
        movementCost
        bodyMotion
        memoryCost
        antennaControlTime
        goalCalcTime
        trialDuration
        weights


    end

    methods
        function obj = CostCalculator()
            %COSTCALCULATOR Construct an instance of this class
            %   Initialise the arrays to be empty
            obj.movementCost = [];
            obj.memoryCost = 0;
            obj.antennaControlTime = table(0, 'VariableNames', {'antennaControlTime'});
            obj.goalCalcTime = table(0, 'VariableNames', {'goalCalcTime'});
            obj.trialDuration = 0;

            
            
            %Movement cost weights
            % Weights
            obj.weights.Neck = 0.5;
            obj.weights.Antenna = [0.04, 0.04, 0.02];
            obj.weights.Mandible = 0.4;
        end

        function obj = addCost(obj, costStruct)
            %ADDCOST Save any values stored in the cost struct to this
            %instance of the class

            if ~isempty(costStruct)
                limbCalculationTime = sum(costStruct.pose.limb.time(:));
                updateContactMemoryTime = sum(costStruct.pose.limb.memory_time(:));
                    
                goalTimeMeasures = fieldnames(costStruct.goal.time); 
                nGoalTimeMeasures = length(goalTimeMeasures);
                sumGoalTime = 0;
                for n = 1:nGoalTimeMeasures
                    sumGoalTime = sumGoalTime + costStruct.goal.time.(goalTimeMeasures{n});
                end


                 obj.antennaControlTime.Variables = obj.antennaControlTime.Variables + limbCalculationTime;
                 obj.goalCalcTime.Variables = obj.goalCalcTime.Variables + sumGoalTime;
                        
                 contactMemoryBytes = costStruct.memory.contact_points;
                 

                 obj.memoryCost = obj.memoryCost + contactMemoryBytes;
            end

        end

        function obj = calculateMotionCost(obj, replayTable)
            %CALCULATEMOTIONCOST From the poses recorded in the replay
            %table at the set sample rate, save the simulation time and the
            %total joint and position costs
            movementCostStruct = obj.weightedMotionCost(obj.weights, replayTable);
            obj.movementCost = [movementCostStruct.joint_motion_total, movementCostStruct.position_cost];
            obj.trialDuration = movementCostStruct.duration;


        end

        function movementStruct = weightedMotionCost(~,weights, replayTable)
            %Sum the change in joint angle according to weights

            % ---------- Find the overall cost for one trial

            %time_s = 0;
            last_pose = replayTable.Pose{1,:};
            last_position = replayTable.Position(1,:);
            nPose = size(replayTable,1);
            nJoint = length(last_pose);

            pose_difference = nan([nPose, nJoint]);
            position_difference = nan([nPose, 4]);


            for i=1:nPose
                %sample_t = replayTable.Time(i);
                pose = replayTable.Pose{i,:};
                position = replayTable.Position(1,:);

                pose_difference(i,:) = abs(pose - last_pose)';
                position_difference(i,:) = abs(position - last_position)';

                last_pose = pose;
                last_position = position;
                %time_s = now;
            end


            %Normalise the weights
            switch class(weights)
                case "struct"
                    weight_mat = [weights.Neck; weights.Neck; 
                        weights.Mandible; weights.Mandible;...
                        weights.Antenna(1); weights.Antenna(2); weights.Antenna(3); ...
                        weights.Antenna(1); weights.Antenna(2); weights.Antenna(3)];
                    
                case "double"
                    if length(weights) == 10
                        weight_mat = weights;
                    else
                        warning("Weights size expected 10x1, instead it is %d x %d - setting to 1", size(weights,1), size(weights,2))
                        weight_mat = ones([10,1]);

                    end

                otherwise
                    warning("Weights are an unexpected file type - setting to 1")
                    weight_mat = ones([10,1]);

            end

            %normalise weights [TODO] currently they are already normalised
            %(They aren't because there aren't only 3 joints, there are 3
            %joint types)
            weights_norm = weight_mat/sum(weight_mat);


            if all(~isnan(pose_difference))
                %calculate the overall cost from start to end of trial
                movementStruct.duration = replayTable.Time(end) - replayTable.Time(1);
                movementStruct.weighted_cost_per_joint = sum(pose_difference,1) .* weights_norm';
                movementStruct.overall_cost = sum(movementStruct.weighted_cost_per_joint);
                movementStruct.joint_motion_per_second = sum(pose_difference,1) / movementStruct.duration;
                movementStruct.joint_motion_total = sum(pose_difference, 1);
                                               
            else
                warning('Pose Change over Trial not detected')
            end

            if all(~isnan(pose_difference))
                movementStruct.position_cost = sum(position_difference, 1);
            end
            
        end


        function costTable = convertToTable(obj, trialRWTime)
            %ConvertToTable Finalise the arrays contained within the class
            %to be in table format
            costTable = table();
            costTable.(1) = obj.movementCost;
            costTable.(2) = obj.memoryCost;

            costTable.(3) = obj.antennaControlTime.Variables;
            costTable.(4) = obj.goalCalcTime.Variables;
            costTable.(5) = obj.trialDuration;
            costTable.(6) = trialRWTime;
            
            costTable.Properties.VariableNames = {'Joint Change', 'Total Contact Memory Bytes', 'antennaControlCalculationTime', 'goalCalcTime', 'Simulation Time' , 'Real World Time' };
                    
            
        end
    end
end