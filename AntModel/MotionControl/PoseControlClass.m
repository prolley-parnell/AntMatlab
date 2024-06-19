classdef PoseControlClass
    %PoseControlCLASS Class that provides joint positions to the Ant
    %Uses an action generator to generate cartesian trajectories, and makes
    %joint waypoints based on those cartesian trajectories.
    %   ChangeLog: 27/09/22 - Persie Rolley-Parnell - Remove the neck
    %   variables to their own class


    properties
        actionGen
        antTree
        interval
        space_limits
        antenna_lock %2x1 array with the value for each antenna set to 1 when the antenna is locked in place

        RUNTIME_ARGS


    end

    methods
        function obj = PoseControlClass(antTree, mandible_max, RUNTIME_ARGS)
            %PoseControlCLASS
            if strcmp(RUNTIME_ARGS.SEARCH.MODE{1}, 'p2p')
                obj.actionGen = SampleActionGen(antTree, mandible_max, RUNTIME_ARGS);
            elseif strcmp(RUNTIME_ARGS.SEARCH.MODE{1}, 'joint')
                obj.actionGen = JointActionGen(RUNTIME_ARGS);
            end
            
            obj.antenna_lock = [0 0];


            obj.antTree = antTree;

            obj.RUNTIME_ARGS = RUNTIME_ARGS;
            obj.interval = RUNTIME_ARGS.RATE;


        end

        function [ant, contactStructArray, poseCostStruct] = updatePose(obj, ant, env, bodyMotionFlag)
            %Update neck and head
            [ant, neckMotionFlag] = obj.moveNeck(ant);
            motionFlag = neckMotionFlag || bodyMotionFlag;
            [ant, contactStructArray, limbCostStruct] = obj.updateLimbs(ant, env, motionFlag);
            poseCostStruct.limb = limbCostStruct;
        end



        % --------- Generalised Limb Update Function
        function [ant, contactStructArray, limbCostStruct] = updateLimbs(obj, ant, env, motionFlag)
            limbCostStruct = struct();

            limbs = ant.limbs;
            contactStructArray = [];

            %[EDIT] For experiment: Mandibles not in use in experiment so
            %do not include them in the limb check (reduce experiment
            %duration)
            %for i=1:length(limbs)
            for i=1:2
                limbCostStruct.name(i) = limbs{i}.type;
                %Move the object rigidBodyTree base to match the global ant
                %position.
                if motionFlag
                    limbs{i} = limbs{i}.updateTreeTransform(ant.q, ant.position);
                end

                ant.plotAnt()
                %If limb is not in collision, store the free point
                [limbs{i}, dataStruct] = obj.tactileSenseEval(limbs{i}, ant.q, ant.position, env);
                
                contactStructArray = cat(1,contactStructArray, [dataStruct{:}]');

                switch(limbs{i}.type)
                    case "Antenna"
                        [obj, limbs{i}, qLocal, moveTime] = obj.moveAntenna(limbs{i}, ant.q, ant.position, ant.oscillator);
                    case "Mandible"
                        [obj, limbs{i}, qLocal, moveTime] = obj.moveMandible(limbs{i}, ant.q, ant.mandible_state);
                    case "Leg"
                        warning("Leg control is not yet implemented");
                end

                %%[COST] Save the movetime cost for each limb
                limbCostStruct.time(i) = moveTime;
                qFull = ant.q;
                %Apply joint mask
                ant.q = limbs{i}.applyMask(qFull, qLocal);


                if ~isempty(dataStruct)
                    ant = ant.addContact(dataStruct);
                    obj.antenna_lock(:) = 0;
                    obj.antenna_lock(limbs{i}.number) = 1;
                    %Update the search space, if this is enabled in the
                    %runtime args
                    %%[COST] Store time taken to save sensed contact points
                    [obj.actionGen, memoryTimeCost] = obj.actionGen.updateContactMemory(ant.contact_points, ant.q, ant.limbs{i}.joint_mask, ant.limbs{i}.number);
                    limbCostStruct.memory_time(i) = memoryTimeCost;
                else
                    limbCostStruct.memory_time(i) = 0;
                end
            end
            ant.limbs = limbs;
            ant.poseController = obj;


        end

        function [limbOut, dataStruct] = tactileSenseEval(obj, limbIn, qIn, positionIn, env)
            dataStruct = cell.empty;

            %Check for collisions
            [limbOut, contact_point, surface_normal] = obj.collisionCheck(limbIn, qIn, env);

            %If limb is not in collision, store the free point
            if ~limbOut.collision_latch
                limbOut.free_point = tbox.findFKglobalPosition(obj.antTree, qIn, positionIn, limbOut.end_effector);
                limbOut.free_pose = qIn(limbOut.joint_mask==1);               
                
            end

            %Print collisions
            obj.showPoints(contact_point, surface_normal, limbOut.colour, 1);
            obj.showPoints(contact_point, surface_normal, limbOut.colour, 2);

            if ~isnan(contact_point)
                if contains(limbOut.name, "Mandible")
                    disp("Mand Contact")
                end
                nCP = size(contact_point,1);
                for i=1:nCP
                    dataStruct{i} = struct("point", contact_point(i,:), "normal", surface_normal(i,:), "limb", limbOut.name);
                end
            end

        end

        % --------- Access functions

        function obj = showPoints(obj, collision_points, surface_normal, colour, figure_n)
            if obj.RUNTIME_ARGS.PLOT.ENABLE(figure_n)
                if ~isnan(collision_points)
                    figure(figure_n)
                    hold on;
                    plot3(collision_points(:,1), collision_points(:,2), collision_points(:,3),'o','MarkerSize',5, 'Color', colour)
                    quiver3(collision_points(:,1),collision_points(:,2),collision_points(:,3), ...
                        surface_normal(:,1),surface_normal(:,2),surface_normal(:,3),'off','color','[0.368 0 0.251]');
                    hold off;
                end
            end
        end

        %------------- Collision Checker

        function [limbOut, collision_points, surface_normals] = collisionCheck(obj, limbIn, q, env)

            collision_points = nan;
            surface_normals = nan([1,3]);

            %objID = 1;
            %distThreshold = 0.09; %This threshold is good for when the
            %velocity limits are 20 degrees per second, less so when 1000 degrees per second
            distThreshold = 0.12; 
            %distThreshold = 0.1; 


            subpose = q(limbIn.joint_mask == 1);
            subtree = limbIn.subtree;
            limbOut = limbIn;
            [~, dist ,witnessPoints] = checkCollision(subtree, subpose, env.objectHandles, 'IgnoreSelfCollision','on', 'Exhaustive', 'off');

            [min_dist, ~] = min(dist);
            [body_i, collision_i] = ind2sub(size(dist), find(dist < distThreshold));

            if any(isnan(dist))
                warning("RigidBodyTree %s intersection with CollisionObject", limbIn.name)
                limbOut.collision_latch = 1;

            elseif and(~isempty(body_i), ~limbIn.collision_latch)
                if obj.RUNTIME_ARGS.TERMPRINT
                    disp("collision!")
                end

                limbOut.collision_latch = 1;

                %[body_i, collision_i] = ind2sub(size(dist), ind);
                nCollision = length(collision_i);
                point_on_obj = nan([nCollision, 3]);
                point_normal = nan([nCollision, 3]);
                for i = 1:nCollision
                    witness_pts = witnessPoints(3*body_i(i)-2:3*body_i(i), 2*collision_i(i)-1:2*collision_i(i));

                    %surface_normals(i,:) = tbox.findNormalCollision(env.FBT{collision_i(i)}, pointOnObj(i,:));
                    [point_on_obj(i,:), point_normal(i,:)] = tbox.findPointOnObjNormalID(env.FBT{collision_i(i)}, witness_pts(:,2)');
                end

                %Prevent contact points that are identical to 3 decimal
                %places
                [~, unique_i] = unique(round(point_on_obj, 3), 'rows', 'stable');
                collision_points = point_on_obj(unique_i,:);
                surface_normals = point_normal(unique_i,:);

            elseif min_dist>=distThreshold
                limbOut.collision_latch = 0;
            end
        end


        % ---------- Antenna functions

        function [obj, antennaOut, qOut, antennaControlTime] = moveAntenna(obj, antennaIn, qIn, globalPosition, ~)
            qOut = qIn;
            antennaOut = antennaIn;
            antennaControlTime = 0;

            if obj.antenna_lock(antennaIn.number)
                if obj.RUNTIME_ARGS.SENSE.ENABLE_BOUNCE_BACK
                    [antennaOut] = obj.actionGen.bounceBack(antennaIn, qIn, globalPosition);
                end
            else
                if or(antennaIn.collision_latch, isempty(antennaIn.trajectory_queue)) 
                    [antennaOut, antennaControlTime] = obj.actionGen.loadAntennaTrajectory(antennaIn, qIn, globalPosition);
                end
    
                [antennaOut, q, successFlag] = tbox.popTrajectory(antennaOut);
                if successFlag
                    qOut = q;
                else
                    warning("You really shouldn't be here - you reloaded the Antenna path and then popped and it still didn't work")
                end
            end
        end

        % function [obj, antennaOut, qOut, antennaControlTime] = moveAntennaOld(obj, antennaIn, qIn, globalPosition)
        %     qOut = qIn;
        %     antennaOut = antennaIn;
        %     antennaControlTime = 0;
        %     %[TODO] If the trajectory is empty, update the joint mean by a
        %     %tiny proportion to match the mean pose of the other antenna
        %     if ~obj.antenna_lock(antennaIn.number)
        %         if or(antennaIn.collision_latch, isempty(antennaIn.trajectory_queue)) 
        %             [antennaOut, antennaControlTime] = obj.actionGen.loadAntennaTrajectory(antennaIn, qIn, globalPosition);
        %         end
        % 
        %         [antennaOut, q, successFlag] = tbox.popTrajectory(antennaOut);
        %         if successFlag
        %             qOut = q;
        %         else
        %             warning("You really shouldn't be here - you reloaded the Antenna path and then popped and it still didn't work")
        %         end
        %     end
        % end


        % -------------- Mandible Functions
        function [obj, mandibleOut, qOut, mandibleTrajTime] = moveMandible(obj, mandibleIn, qIn, antMandibleState)
            qOut = qIn;
            mandibleOut = mandibleIn;
            mandibleTrajTime = 0;

            if obj.RUNTIME_ARGS.BODY_MOTION_ENABLE
                if and(antMandibleState ~= mandibleIn.motion_state, ~mandibleIn.collision_latch)
                    %Make a new trajectory
                    mandibleIn.motion_state = antMandibleState;

                    %[TODO] [COST] Add time taken to load a mandible
                    %trajectory
                    [mandibleOut, reloadSuccessFlag] = tbox.loadMandibleTrajectory(mandibleIn, qIn, obj.actionGen.maxvelocities, obj.interval);

                    if ~reloadSuccessFlag
                        return
                    end
                end


                if mandibleOut.motion_state ~= 0
                    [mandibleOut, q, successFlag] = tbox.popTrajectory(mandibleOut);
                    if successFlag
                        qOut = q;
                    else
                        if isempty(mandibleOut.trajectory_queue)
                            mandibleOut.motion_state = 0;
                        else
                            warning("You really shouldn't be here - you reloaded the Mandible path and then popped and it still didn't work")
                        end
                    end
                end
            end
        end

        % ------------ Neck Pose update function

        function [neckOut] = newNeckTrajectory(obj, neckIn, qIn, goalStruct)
            neckOut = neckIn;
            if obj.RUNTIME_ARGS.BODY_MOTION_ENABLE
                waypoints = tbox.generateNeckTrajectory(neckIn, qIn, goalStruct);
                %Turn the sequential 4x4 poses into joint configurations
                neckOut.trajectory_queue = neckIn.findIKforNeckPose(waypoints, qIn);
            end
        end

        function [antOut, successFlag] = moveNeck(~, antIn)
            antOut = antIn;
            qIn = antIn.q;
            neckIn = antIn.neckObj;

            [neckOut, qLocal, successFlag] = tbox.popTrajectory(neckIn);
            if successFlag
                antOut.neckObj = neckOut;
                antOut.q = neckOut.applyMask(qIn, qLocal);
            end

        end



    end
end

