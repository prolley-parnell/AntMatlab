classdef InformationGain
    %INFORMATIONGAIN Used to evaluate a space in which to search and
    %identify what cartesian point is best to search next

    properties (SetAccess = public, GetAccess = public)
        information_measures
        n_sample

        cp
        cn
        sig1
        sig3
        sigalpha
        mu3

        mand_max


    end

    methods
        function obj = InformationGain(RUNTIME_ARGS)
            %INFORMATIONGAIN Construct an instance of this class
            %   Use the runtime args to define which components are
            %   considered in information gain
            measureArg = RUNTIME_ARGS.SEARCH.REFINE.ARG;
            nMeasureArg = length(measureArg);
            measureIGEF = {'psi1', 'psi2', 'psi3', 'psi4'};
            measureFlag = zeros([nMeasureArg,4]);
            for t=1:nMeasureArg
                measureFlag(t,:) = strcmp(measureArg{t}, measureIGEF);
                if ~any(measureFlag(t,:))
                    warning('Information Gain argument "%s" not implemented', measureArg{t})
                end
            end
            obj.information_measures = any(measureFlag,1);

            %number of samples to evaluate for finding the highest information gain
            obj.n_sample = 5;
    
            %Parameters set by Active Tactile Exploration Based on
            %Cost-Aware Information Gain Maximisation
            obj.sig1 = RUNTIME_ARGS.SEARCH.REFINE.PARAM(1);
            obj.sig3 = RUNTIME_ARGS.SEARCH.REFINE.PARAM(2);
            obj.sigalpha = RUNTIME_ARGS.SEARCH.REFINE.PARAM(3);
            obj.mu3 = RUNTIME_ARGS.SEARCH.REFINE.PARAM(4);

        end

        function goalOut = refine(obj, goalPropStruct)
            %REFINE apply the method of refinement appropriate to
            %InformationGain to reduce a set of goal samples to a single
            %goal
            goalOut = goalPropStruct(1);
            %Deal with exceptions (no contact points, or no refinement
            %measures)
            nMeasures = length(obj.information_measures);
            if nMeasures < 1
                warning("No refinement measures defined, selecting goal 1")
                return
            end

            nGoal = length(goalPropStruct);
            psi1 = ones([nGoal,1]);
            psi2 = ones([nGoal,1]);
            psi3 = ones([nGoal,1]);
            psi4v2 = ones([nGoal,1]);
            for i=1:nGoal

                %For each goal, get the information gain measure
                if obj.information_measures(1)
                    psi1(i) = obj.calculatePsi1(goalPropStruct(i).endPt);
                end
                if obj.information_measures(2)
                    psi2(i) = obj.calculatePsi2(goalPropStruct(i).cartesianPath);
                end
                if obj.information_measures(3)
                    psi3(i) = obj.calculatePsi3(goalPropStruct(i).endPt);
                end
                if obj.information_measures(4)
                    psi4v2(i) = obj.calculatePsi4v2(goalPropStruct(i).jointPath);
                end

            end
            [~, goalIndex] = max(psi1.*psi2.*psi3.*psi4v2);
            goalOut = goalPropStruct(goalIndex);

        end

        function obj = setContactMemory(obj, contact_pointStruct)

            obj.cp = cat(1,contact_pointStruct(:).point);
            obj.cn = cat(1,contact_pointStruct(:).normal);

        end
        

        function psi1 = calculatePsi1(obj, goalSample)

            exp_input = vecnorm(goalSample - obj.cp, 2, 2).^2;
            allpsi1 = 1 - exp(-exp_input/(obj.sig1.^2));
            psi1 = min(allpsi1);

        end
        function psi2 = calculatePsi2(~, goalCartesianPath)
            %Current path considered is the direct distance and not the
            %true trajectory

            nWaypoint = size(goalCartesianPath, 1);
            totalPositionChange = zeros([1, size(goalCartesianPath,2)]);
            for n = 1: nWaypoint-1
                positionChange = abs(goalCartesianPath(n,:) - goalCartesianPath(n+1,:));
                totalPositionChange = totalPositionChange + positionChange;
            end
            pathLength = vecnorm(totalPositionChange,2,2);
            psi2 = 1 / pathLength;

        end
        function psi2v2 = calculatePsi2v2(~, currentPosition, goalPosition)
            %Current path considered is the direct distance and not the
            %true trajectory
            pathLength = vecnorm(goalPosition-currentPosition,2,2);
            psi2v2 = 1 / pathLength;

        end

        function psi3 = calculatePsi3(obj, goalSample)

            exp_input = (vecnorm(goalSample - obj.cp, 2, 2) - obj.mu3).^2;
            allpsi3 = exp(-exp_input / (obj.sig3.^2));
            psi3 = sum(allpsi3);

        end

        function psi4v2 = calculatePsi4v2(obj, goalJointTraj)
            %Sum the variation in joint pose across the trajectory
            nPose = size(goalJointTraj,2);
            totalJointChange = zeros([size(goalJointTraj,1),1]);
            for n = 1: nPose-1
                jointChange = abs(goalJointTraj(:,n) - goalJointTraj(:,n+1));
                totalJointChange = totalJointChange + jointChange;
            end
            psi4v2 = exp(-sum(totalJointChange)/(obj.sigalpha)^2);
                
        end
    end
end

