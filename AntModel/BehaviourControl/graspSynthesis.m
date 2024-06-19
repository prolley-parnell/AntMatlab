classdef graspSynthesis
    %GRASPSYNTHESIS Given the contact points, select a grasp location
    %Changelog - created 03/11/2022
    properties

        mandible_max
        mandible_depth
        synth_method
        enforce_mand_max
        %exhaustive_search_flag

        sc_count

        quality
        COC

        RUNTIME_ARGS

    end

    methods
        function obj = graspSynthesis(RUNTIME_ARGS, maxMandibleDist, mandibleDepth)
            %GRASPSYNTHESIS Construct an instance of this class

            obj.mandible_max = maxMandibleDist;
            obj.mandible_depth = mandibleDepth;
            obj.enforce_mand_max = RUNTIME_ARGS.SENSE.MAND_MAX;

            methodArg = RUNTIME_ARGS.SENSE.MODE; %Cell array of the different qualities used to select a grasp

            nMethodArg = length(methodArg);
            availableMethod = {'dist', 'align', 'PCA', 'last'};
            nAvailableMethod = length(availableMethod(:));
            measureFlag = zeros([nMethodArg, nAvailableMethod]);
            for t=1:nMethodArg
                measureFlag(t,:) = strcmpi(methodArg{t}, availableMethod);
                if ~any(measureFlag(t,:))
                    warning('Grasp Synthesis Measure "%s" not implemented', methodArg{t})
                end
            end
            obj.synth_method = any(measureFlag,1);

            % Stopping criteria counter, counts the number of time steps
            % that the stopping criteria is past the set threshold
            obj.sc_count = 0;


            obj.RUNTIME_ARGS = RUNTIME_ARGS;

            %Measure of quality of the current goal position.
            obj.quality = RUNTIME_ARGS.SENSE.THRESH;

            %Centre of contacts, the mean value of each of the contact
            %points collected through tactile sensing
            obj.COC = struct("mean", [], "delta", []);


        end

        function [antOut, goalOut, goalCostStruct] = check(obj, antIn, sensedData, env)
            %CHECK Use the methods defined to evaluate the contact point data pairs and select a goal

            goalCostStruct = struct(); %Define the table used to store cost info

            goalOut = goalStruct();
            antOut = antIn;

            %Deal with exceptions (no contact points, or no grasp synthesis method)
            %% [COST] Start exhaustive grasp search
            senseTStart = tic;
            if isempty(sensedData)
                %[COST] End time if no new data has been collected
                graspSynthTEnd = toc(senseTStart);
                goalCostStruct.time.grasp_synth = graspSynthTEnd;
                return
            else
                nMeasures = length(obj.synth_method);
                if nMeasures < 1
                    warning("No grasp synthesis method selected, goal cannot be generated");
                    %[COST] End time measure if no grasp selection method
                    %is selected
                    graspSynthTEnd = toc(senseTStart);
                    goalCostStruct.time.grasp_synth = graspSynthTEnd;
                    return
                end
            end
            % obj = obj.calcCOC(antIn.contact_points);
            [obj, stop_flag] = obj.evaluateStoppingCriterion(antIn.contact_points);
            if ~stop_flag
                %[COST] Record time up to point where not enough contacts
                %have been gathered to gen goal
                antOut.graspGen = obj;
                graspSynthTEnd = toc(senseTStart);
                goalCostStruct.time.grasp_synth = graspSynthTEnd;
                return
            end
            
            if obj.synth_method(3)
                obj = obj.calcCOC(antIn.contact_points);
                [graspAxesOut] = obj.findPCAGraspAxes(antIn);
            
            elseif obj.synth_method(4)
                obj = obj.calcCOC(antIn.contact_points);
                [graspAxesOut] = obj.findLastContactGraspAxes(antIn);
            else
                [graspAxesOut] = obj.findExhaustiveGoalAxes(antIn);
            end

            graspSynthTEnd = toc(senseTStart);
            goalCostStruct.time.grasp_synth = graspSynthTEnd;
            % [COST] End exhaustive grasp search

            [goalOut] = obj.approximateGrasp(graspAxesOut, env);

            antOut = obj.mandibleMotionStateMachine(antIn, sensedData);

            antOut.graspGen = obj;


        end
        %This may not work and may not update the object
        function [obj, stop_flag] = evaluateStoppingCriterion(obj, contact_point_struct)
            %EVALUATESTOPPINGCRITERION determine based on the RuntimeArgs
            %whether the sensed information is enough to calculate a goal.

            %Output: stop_flag - If true then stop the process and evaluate the
            %currently sensed data for a grasp location, else continue to sense
            stop_flag = 0;

            enabled_sc = {'n_contact', 'coc', 'distance'};
            sc_mode_flag = strcmpi(obj.RUNTIME_ARGS.STOP.MODE, enabled_sc);

            if sc_mode_flag(1) %Stop based on the number of contacts
                nContactThresh = length(contact_point_struct) - obj.RUNTIME_ARGS.STOP.THRESH;
                if nContactThresh >= 0
                    obj.sc_count = obj.sc_count + 1;
                else
                    obj.sc_count = 0;
                end

            elseif sc_mode_flag(2) %Stop when the change in COC is below a threshold
                obj = obj.calcCOC(contact_point_struct);
                if obj.COC.delta < obj.RUNTIME_ARGS.STOP.THRESH
                    obj.sc_count = obj.sc_count + 1;
                else
                    obj.sc_count = 0;
                end
            elseif sc_mode_flag(3) %Stop when the last two contact points are less than mandible distance apart
                
                dist = obj.getLastAntennaDistance(contact_point_struct);
                if dist < obj.mandible_max
                    obj.sc_count = obj.sc_count + 1;
                else
                    obj.sc_count = 0;
                end
            else
                warning("No stopping criterion has been selected, a grasp may never be evaluated")
            end
            if obj.sc_count >= obj.RUNTIME_ARGS.STOP.COUNT
                stop_flag = 1;
            end

        end
        
        function [axesOut] = findLastContactGraspAxes(obj, antIn)
        %FINDLASTCONTACTGRASPAXES Use the last two left and right antennal
        %contacts to find the grasp.

            [left_point, right_point] = obj.getLastAntennaContacts(antIn.contact_points);
            axesOut = obj.calculateApproachAxis([left_point, right_point], antIn.position);

        
        end

        function [axesOut] = findPCAGraspAxes(obj, antIn)
            %FINDPCAGRASP Use the contact points in memory to generate a
            %grasp axis for the ant approach
            % Input:
            % AntIn - Ant Object containing all contact points in memory
            % env - CollisionObjects class with object triangulation
            % Output:
            % axesOut - consists of the desired midpoint of the grasp,
            % the axis used to approach that point (Y axis), Z, and X axes

            cartPointArray = cat(1,antIn.contact_points(:).point);

            % Step 1: Calculate the covariance matrix
            CM = cov(cartPointArray);
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

            % Define X axis (perpendicular to the most variance compared with global Z)
            x_axis = cross(var_axis, [0 0 1]);
            x_axis_n = x_axis/norm(x_axis);


            % Define Y axis (PC with most variance)
            y_axis_n = var_axis/norm(var_axis);

            % Define Z axis (to find right angle grasp frame)
            z_axis = cross(x_axis_n, y_axis_n);
            z_axis_n = z_axis/norm(z_axis);

            axesOut = struct('X', x_axis_n, 'Y', y_axis_n, 'Z', z_axis_n, 'MP', obj.COC.mean);

        end

        function [axesOut, bestQuality] = findExhaustiveGoalAxes(obj, antIn)
            %FINDEXHAUSTIVEGOALAXES Check all contact points stored in ant
            %memory and compare with every other point to find the best
            %grasp according to the grasp selection criteria
            % Criteria: 'align', 'dist', 'align and dist'
            % Can enable a mask that disqualifies any contact points that
            % are too far apart (greater than the mandible reach)
            %Cartesian contact points
            cartPointArray = cat(1,antIn.contact_points(:).point);
            nContactPoint = length(antIn.contact_points);
            distanceMat = ones(nContactPoint);
            mandMaxFlag = ones(nContactPoint);
            alignMeasure = ones(nContactPoint);

            %For each goal, get the information gain measure
            if obj.synth_method(1) %Distance
                [distanceMat, ~, ~] = obj.findInterPointDistance(cartPointArray, "none");
                %Normalise the distance mat to be between 0 and 1
                %distMeasure = rescale(distanceMat);
            end
            if obj.enforce_mand_max
                if ~obj.synth_method(1)
                    [distanceMat, ~, ~] = obj.findInterPointDistance(cartPointArray, "none");
                end
                mandMaxFlag = obj.mandibleLimit(distanceMat);
            end
            if obj.synth_method(2) %Alignment
                [alignMeasure, ~, ~] = obj.findInterPointGraspAlign(antIn.contact_points);
            end

            [bestQuality, goalIndex] = max(distanceMat.*mandMaxFlag.*alignMeasure,[],'all');

            [a, b] = ind2sub([nContactPoint, nContactPoint], goalIndex);

            if obj.RUNTIME_ARGS.PLOT.ENABLE(1)
                contact_points = cat(1,antIn.contact_points([a,b]).point);
                figure(1)
                hold on
                plot3(contact_points(:,1),contact_points(:,2),contact_points(:,3), 'r', 'LineStyle', ':');
                hold off
            end
            axesOut = obj.calculateApproachAxis(antIn.contact_points([a,b]), antIn.position);

        end

        function goalOut = approximateGrasp(obj, desiredGraspAxes, env)
            %APPROXIMATEGRASP Given either a desired set of mandible
            %contacts, or an axis of approach, select the true mandible
            %collision points
            % Input:
            % desiredGraspAxes: struct containing the X,Y,Z axes and MP.
            % All axes point away from the MP
            % env: CollisionObjects class instance containing
            % triangulations for objects in the environment
            % Output:
            % goalOut: instance of goalStruct where the grasp contacts have
            % been set based on the provided axis of approach in
            % desiredGraspAxes

            goalOut = goalStruct();
            includeOrigin = 1;

            %Set up an empty contact point struct
            contactStruct = struct("point", [], "normal", []);
            contactPts = repmat(contactStruct, [2,1]);


            %1: find the point of intersection along the X axis
            startPt = desiredGraspAxes.MP;
            approachRay = -desiredGraspAxes.Y;

            %If either point makes contact, the contact with the distance
            %closest to the starting approachTip is the first contact
            %Use 'line' argument to consider if points are on either side of
            %the mandible tip along the axis of approach
            expandBorder = {'border','inclusive'};
            % infiniteLine = {'LineType', 'line'};

            % Use a ray to point along the Y axis, away from the MP
            %[mand_base_contact_pt, ~, ~] = env.findRayIntersect(startPt, -approachRay, includeOrigin, expandBorder);

            % Approach shape along Y axis
            % Find the midpoint of the mandibles at the furthest possible
            % pose while
            %approachTipMP = mand_base_contact_pt - (approachRay * obj.mandible_depth);
            approachTipA = (startPt  + (desiredGraspAxes.X * obj.mandible_max*0.5)) - 5 * approachRay ; %Mandible tips at closest possible contact
            approachTipB = (startPt  - (desiredGraspAxes.X * obj.mandible_max*0.5)) - 5 * approachRay ;


            % Project from the furthest points of possible contact along
            % the ray of approach to find any points of collision
            [contactPts(1).point, contactPts(1).normal, ~, distA] = env.findRayIntersect(approachTipA, approachRay, includeOrigin, expandBorder);
            [contactPts(2).point, contactPts(2).normal, ~, distB] = env.findRayIntersect(approachTipB, approachRay, includeOrigin, expandBorder);

            if obj.RUNTIME_ARGS.PLOT.ENABLE(1)
                figure(1)
                hold on
                quiver3(approachTipA(1), approachTipA(2), approachTipA(3), approachRay(1), approachRay(2), approachRay(3), 'LineWidth', 3, 'Color', 'c')
                quiver3(approachTipB(1), approachTipB(2), approachTipB(3), approachRay(1), approachRay(2), approachRay(3), 'LineWidth', 3, 'Color', 'c')
                hold off
            end

            finiteFlag = isfinite([distA, distB]);
            if all(~finiteFlag)
                %Approach rays do not collide with the object therefore go
                %to point where the mandible base is closest and close
                %mandibles

                % Use a ray to point along the Y axis, away from the MP
                [mand_base_contact_pt] = env.findRayIntersect(startPt, -approachRay, includeOrigin, expandBorder);

                %Find the mandible tips at the closest position
                closestMandBase = mand_base_contact_pt + (approachRay * obj.mandible_depth);
                tipA = closestMandBase + (desiredGraspAxes.X * obj.mandible_max*0.5); %Mandible tips at closest possible contact
                tipB = closestMandBase - (desiredGraspAxes.X * obj.mandible_max*0.5);

                %Draw the arcs where the ray points from the open position
                %to the closed position
                rayA = tipB - tipA;
                rayB = -rayA;


                [contactPts(1).point, contactPts(1).normal, ~] = env.findRayIntersect(tipA, rayA, includeOrigin, expandBorder);
                [contactPts(2).point, contactPts(2).normal, ~] = env.findRayIntersect(tipB, rayB, includeOrigin, expandBorder);

            else
                %Check for signed vector (-ve means the collision occurs
                %further away than the estimated closest pose without
                %collision)
                [~, closeID] = min([distA, distB]);
                if closeID == 1
                    tipA = contactPts(1).point - (desiredGraspAxes.X * obj.mandible_max);
                    closeRay = desiredGraspAxes.X;
                    [contactPts(2).point, contactPts(2).normal, ~, ~] = env.findRayIntersect(tipA, closeRay, ~includeOrigin, expandBorder);
                else
                    tipB = contactPts(2).point + (desiredGraspAxes.X * obj.mandible_max);
                    closeRay = -desiredGraspAxes.X;
                    [contactPts(1).point, contactPts(1).normal, ~, ~] = env.findRayIntersect(tipB, closeRay, ~includeOrigin, expandBorder);

                end

            end

            %5: Set the contact points in the goal struct
            goalOut = goalOut.setGraspContact(contactPts);
            goalOut = goalOut.saveGoalApproach(desiredGraspAxes);

        end


        function [axesOut] = calculateApproachAxis(obj, contactPointPair, positionIn)
            %CALCULATEAPPROACHAXIS From a set of contact points, calculate
            %the axis of approach to grasp
            % Used specifically for point to point grasp calculations
            % Input:
            % contactPointPair: 2x1 struct containing the relevant contact
            % points (only cartesian position is needed)
            % antPosition: 1x4 array showing the position of the ant at the
            % current time step
            % - Update from function goalStruct.SetGoal 21/02/23

            pointArray = cat(1,contactPointPair(:).point);

            x_axis = pointArray(2,:) - pointArray(1,:);
            x_axis_n = x_axis/norm(x_axis);

            midpoint = mean(pointArray, 1);

            global_z_vector = [0 0 1];
            y_axis = cross(x_axis_n, global_z_vector);
            y_axis_n = y_axis/norm(y_axis);

            z_axis = cross(x_axis_n, y_axis_n);
            z_axis_n = z_axis/norm(z_axis);

            axesOut = struct('X', x_axis_n, 'Y', y_axis_n, 'Z', z_axis_n, 'MP', midpoint);
            axesOut = obj.checkPositionAlignment(axesOut, positionIn);

            if obj.RUNTIME_ARGS.PLOT.ENABLE(1)
                figure(1)
                hold on
                quiver3(axesOut.MP(1),axesOut.MP(2),axesOut.MP(3), axesOut.X(1),axesOut.X(2),axesOut.X(3), 'Color','r');
                quiver3(axesOut.MP(1),axesOut.MP(2),axesOut.MP(3), axesOut.Y(1),axesOut.Y(2),axesOut.Y(3), 'Color','g');
                quiver3(axesOut.MP(1),axesOut.MP(2),axesOut.MP(3), axesOut.Z(1),axesOut.Z(2),axesOut.Z(3), 'Color','b');
                hold off
            end
        end

        function [axesOut] = checkPositionAlignment(~, axesIn, positionIn)
            %CHECKALIGNMENT Approach desired goal from the direction that
            %points towards the ants current position
            %If the distance between the MP+Y axis is closer than
            %MP-Y axis then invert alignment, also ensure Z axis is always
            %pointing up
            % Input:
            % axesIn: Struct with properties X,Y,Z and MP. Grasp is
            % approached along -Y axis
            % positionIn: 4x1 array with the ant current position
            % Output:
            % axesOut: Struct with properties X,Y,Z and MP. Y axis is
            % modified to point away from the centre of contacs, and Z axis
            % forced to be positive

            %[EDIT] Now finding axis pointing furthest from COC
            axesOut = axesIn;
            oldAlignment = axesIn.Y;

            PosToMP = axesIn.MP - positionIn(1:3);
            oldDist = vecnorm(PosToMP + oldAlignment, 2,2);
            newDist = vecnorm(PosToMP - oldAlignment, 2,2);


            if newDist < oldDist %
                axesOut.Y = -oldAlignment;
                %Maintain the right handed coordinate system and flip the X
                %as well
                axesOut.X = -axesIn.X;
            end

            % Ensure Z axis is positive
            axesOut.Z = sign(axesIn.Z(3))*axesIn.Z;

        end
        function [axesOut] = checkAlignment(obj, axesIn)
            %CHECKALIGNMENT Approach desired goal from the direction that
            %points away from the Centre of Contacts
            %If the distance between the MP+Y axis is closer than
            %MP-Y axis then invert alignment, also ensure Z axis is always
            %pointing up
            % Input:
            % axesIn: Struct with properties X,Y,Z and MP. Grasp is
            % approached along -Y axis
            % positionIn: 4x1 array with the ant current position
            % Output:
            % axesOut: Struct with properties X,Y,Z and MP. Y axis is
            % modified to point away from the centre of contacs, and Z axis
            % forced to be positive

            %[EDIT] Now finding axis pointing furthest from COC
            axesOut = axesIn;
            oldAlignment = axesIn.Y;

            COCToMP = axesIn.MP - obj.COC.mean;
            oldDist = vecnorm(COCToMP + oldAlignment, 2,2);
            newDist = vecnorm(COCToMP - oldAlignment, 2,2);


            if newDist > oldDist %
                axesOut.Y = -oldAlignment;
                %Maintain the right handed coordinate system and flip the X
                %as well
                axesOut.X = -axesIn.X;
            end

            % Ensure Z axis is positive
            axesOut.Z = sign(axesIn.Z(3))*axesIn.Z;

        end


        function [flagArray] = mandibleLimit(obj, distanceArray)

            flagArray = ones(size(distanceArray));
            flagArray(distanceArray>obj.mandible_max) = 0;

        end

        function antOut =  mandibleMotionStateMachine(~, antIn, ~)
            mandibleContactCheck = [];
            for l = 1:length(antIn.limbs)
                if contains(antIn.limbs{l}.name, "Mandible")
                    mandibleContactCheck(end+1) = antIn.limbs{l}.collision_latch;
                end
            end
            %mandibleContactCheck = sum(contains([sensedData(:).limb], "Mandible"));
            antOut = antIn;
            if mandibleContactCheck > 1
                antOut.grasp_complete = 1;
                antOut.mandible_state = 0;
                antOut.positionController.trajectory_queue = [];
            elseif mandibleContactCheck == 1
                antOut.positionController.trajectory_queue = [];
                antOut.mandible_state = 1;
            end
        end



        function [alignMat, bestAlign, idx] = findInterPointGraspAlign(obj, contactStruct)
            %findInterPointGraspAlign find the alignment at a point of contact
            %with the surface vector for a given opposing contact

            pointArray = cat(1,contactStruct(:).point);
            normArray = cat(1,contactStruct(:).normal);
            nContact = size(pointArray,1);
            alignMat = nan([nContact, nContact]);
            idx = [];

            %[distanceMAT, ~, ~] = obj.findInterPointDistance(pointArray, type);
            for n = 1:nContact
                for m = 1:nContact
                    if n~= m
                        contactPair = pointArray([n, m],:);
                        forcePair = obj.genOpposeForces(contactPair);
                        normalPair = normArray([n,m],:);

                        alignPair = min(tbox.findSurfNormAlign(normalPair, forcePair));
                        alignMat(n,m) = alignPair;
                        alignMat(m,n) = alignPair;
                    end

                end


            end
            bestAlign = max(alignMat,[], 'all');
            [idx(:,1), idx(:,2)] = find(alignMat == bestAlign);

        end
        function obj = calcCOC(obj, contactStruct)
            new_COC = mean(cat(1,contactStruct(:).point),1);
            obj.COC.delta = inf;
            if ~isempty(obj.COC.mean)
                delta = sqrt(sum((obj.COC.mean - new_COC).^2));
                obj.COC.delta = delta;

                if obj.RUNTIME_ARGS.PLOT.ENABLE(2)
                    figure(2)
                    hold on;
                    plot3([obj.COC.mean(1) ; new_COC(1)],[obj.COC.mean(2) ; new_COC(2)] ,[obj.COC.mean(3); new_COC(3)], '+--')
                    hold off;
                end
            end

            %[TODO] Add the ability to have a running mean rather than
            %storing all contact points (see jointActionGen.addPoseToMean)
            obj.COC.mean = new_COC;


        end


        function [distanceMAT, distance, idx] = findInterPointDistance(~, contact_points, type)


            distanceMAT = pdist2(contact_points, contact_points);
            distanceMAT(distanceMAT==0) = nan;


            switch type
                case "min"
                    distance = min(distanceMAT, [], 'all');

                case "max"
                    distance = max(distanceMAT, [], 'all');

                otherwise
                    distance = nan;
                    %Min or Max not selected - no problem
            end
            [row, col] = find(distanceMAT == distance);
            if and(~isempty(row), ~isempty(col))
                idx = [row(1), col(1)];
            else
                idx = [];
            end
        end

        function [forces] = genOpposeForces(~, contacts)
            %Generate a pair of forces that point in the direction of
            %the other contact point as if pinched in a vice
            contactA = contacts(1,:);
            contactB = contacts(2,:);
            force_a = contactB - contactA;
            forceA_norm = force_a/vecnorm(force_a);
            forceB_norm = -forceA_norm;
            forces = [forceA_norm;forceB_norm];

        end

        function [distance] = getLastAntennaDistance(~, contact_points)
            %% [TODO] This doesn't ignore mandible contacts)
            name = cat(2, contact_points.limb);
            side = split(name, "_");
            
            left_idx = contains(side(:,:,1), "Left");
            right_idx = contains(side(:,:,1), "Right");

            if any(left_idx)
                last_left_idx = find(left_idx,1, 'last');
                left_contact = contact_points(last_left_idx);
            else
                distance = Inf;
                return;
            end
            
            if any(right_idx)
                last_right_idx = find(right_idx,1, 'last');
                right_contact = contact_points(last_right_idx);
            else
                distance = Inf;
                return;
            end

            distance = vecnorm(left_contact.point - right_contact.point);

        end

        function [left_contact, right_contact, distance] = getLastAntennaContacts(~, contact_points)
            %% [TODO] This doesn't ignore mandible contacts)
            name = cat(2, contact_points.limb);
            side = split(name, "_");
            
            left_idx = contains(side(:,:,1), "Left");
            right_idx = contains(side(:,:,1), "Right");
            
            left_contact = NaN;
            right_contact = NaN;
            if any(left_idx)
                last_left_idx = find(left_idx,1, 'last');
                left_contact = contact_points(last_left_idx);
            else
                distance = Inf;
                return;
            end
            
            if any(right_idx)
                last_right_idx = find(right_idx,1, 'last');
                right_contact = contact_points(last_right_idx);
            else
                distance = Inf;
                return;
            end

            distance = vecnorm(left_contact.point - right_contact.point);

        end

    end
end

