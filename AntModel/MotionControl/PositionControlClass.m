classdef PositionControlClass
    % POSITIONCONTROLCLASS Class to update the position [x y z yaw] of the rigidBodyTree antTree
    % Position [x y z yaw] is used to place the rigid body tree in the simulated
    % environment.
    % This class also generates trajectories between global navigation
    % locations.
    % ChangeLog - Emily Rolley-Parnell 12/09/2022 - Started introducing
    % code to follow the path according to velocity limits.
    %
    % POSITIONCONTROLCLASS Properties:
    %
    %   goal - an instance of goalStruct containing the position to
    %          grasp and the heading vector - this is the goal the ant is
    %          currently navigating to.
    %
    %     bodyTree - A copy of the full ant RigidBodyTree variable
    %
    %     trajectory_queue - A queue of [x y z yaw] to be popped off at each time step
    %
    %     map - A binary occupation map used in path planning and avoiding environment collisions.
    %
    %     RUNTIME_ARGS - Full struct copy of the runtime arguments for this trial.
    %
    % POSITIONCONTROLCLASS Methods:
    %     PositionControlClass - Class definition
    %
    %     updatePosition - Update global position from the queue
    %
    %     updateGoal - Call makeMap and setGoal
    %
    %     makeMap - Define an occupancy map of the local area for navigating to the goal using collision data. [Currently only
    %     updated when new goal is defined]
    %
    %     [TODO REMOVE] setGoal - Assign values to the goal struct based on the
    %     evaluated goal
    %
    %     loadTrajectory - Use the goal struct property and an approach
    %     mode to generate path waypoints and a trajectory between those
    %     points. This trajectory is added to the queue property.
    %
    %     generateTrapTraj - Generate a set of global cartesian waypoints
    %     that fit in a trapezoidal velocity curve based of the global
    %     position velocity limits.
    %
    %     generateLineTraj - Generate a set of global cartesian waypoints
    %     with a fixed velocity to define the inter point distance.
    %
    %     trajFromAstar - Use the occupancyMap and the start position and
    %     goal position to plan a collision-free path.
    %
    %     popTrajectory - Returns the position [x y z yaw] value from the top
    %     of the queue and removes it from the queue.
    %
    %     findYawAngle - Find the angle of rotation about the Z axis to map
    %     from the X axis to the given vector. The Z component of the given
    %     vector is ignored.
    %
    %     checkAlignment - Find the goal heading vector that results in the
    %     smallest change in global position.
    %
    %     transformGoalMandNeck - Find the goal position relative to the
    %     neck joint, given a goal that is relative to the mandible base.
    %
    %     plotGoal - Visualise the goal struct in the virtual environment.
    %
    % See also POSECONTROLCLASS

    properties

        goal % Global frame navigation desired endpoint - updated when better goal is found.
        bodyTree % A copy of the full ant RigidBodyTree variable
        trajectory_queue % A queue of [x y z yaw] to be popped off at each time step.
        map % A binary occupation map used in path planning and avoiding environment collisions.
        RUNTIME_ARGS % Full struct copy of the runtime arguments for this trial.
        controller %a Pure Pursuit Controller for following the trajectory queue
        linear_vel %Linear velocity limit of the ant head
        angular_vel %angular velocity limit of the ant head
        mandible_max % The maximum distance between the two mandble halves

    end

    methods
        function obj = PositionControlClass(antTree, mandible_max, RUNTIME_ARGS)

            %POSITIONCONTROLCLASS Create an instance of the class

            obj.goal = goalStruct; %Initialise goal as empty

            obj.bodyTree = antTree; %Make a copy of the full body ant tree

            obj.trajectory_queue = []; %Initialise queue as empty

            obj.RUNTIME_ARGS = RUNTIME_ARGS; %Make a copy of the runtime arguments

            obj.linear_vel = RUNTIME_ARGS.BODY_VEL_LIMITS.LINEAR * RUNTIME_ARGS.RATE;
            %Angular velocity Default: 1.57 radians per second
            obj.angular_vel = RUNTIME_ARGS.BODY_VEL_LIMITS.ANGULAR * RUNTIME_ARGS.RATE;
            obj.controller = controllerPurePursuit('DesiredLinearVelocity', obj.linear_vel, 'MaxAngularVelocity', obj.angular_vel);
            obj.mandible_max = mandible_max;

        end
        
        function [obj, positionOut, successFlag, positionCostStruct, graspFlag] = neutralPosition(obj, positionIn, qIn, last_contacts)
            %UPDATEPOSITION Get the next position from the queue
            % If the queue is empty and there is a goal, then generate
            % another trajectory to be added to the queue then try again.
            %
            % Function Properties:
            %   obj - POSITIONCONTROLCLASS Class object
            %
            %   positionIn - [x y z yaw] of the ant at the current time step.
            %
            %   qIn - 10x1 vector of the joint angles of the rigidBodyTree
            %   at the current time step.
            %
            %   last_contacts - The last antenal contacts left and right
            %   made by the antennae
            %[COST] PositionCost INIT [TODO]
            positionCostStruct = struct.empty();
            graspFlag = 0;
            
            if obj.RUNTIME_ARGS.BODY_MOTION_ENABLE && ~isempty(last_contacts.left) && ~isempty(last_contacts.right)
                contact_ipd = vecnorm(last_contacts.left.point - last_contacts.right.point); 
                if contact_ipd < obj.mandible_max 
                    graspFlag = 1;
                    %If contact has been made with the left antenna, then find
                    %the distance between the current mandible left tip and the
                    %target last location
                    left_cartesian = tbox.findFKglobalPosition(obj.bodyTree, qIn, positionIn, 'left_jaw_tip');
                    left_offset = last_contacts.left.point - left_cartesian;
                    left_dist = vecnorm(left_offset);
                    right_cartesian = tbox.findFKglobalPosition(obj.bodyTree, qIn, positionIn, 'right_jaw_tip');
                    right_offset = last_contacts.right.point - right_cartesian;
                    right_dist = vecnorm(right_offset);
        
                    % Find the rotation so that each point is an equal distance
                    % from the mandible tip (rotate towards the side that is
                    % further
                    if right_dist > left_dist
                        % Rotate to the left
                        rotation = min((right_dist - left_dist) / (right_dist + left_dist), obj.angular_vel);
                    else
                        % Rotate to the right
                        rotation = -min((left_dist - right_dist) / (right_dist + left_dist), obj.angular_vel);
                    end
    
                    if abs(rotation) < obj.angular_vel
                        rotation = 0;
                    end
        
                    % Move the body so that the Distance to the left tip and
                    % distance to the right tip is halved or smaller
                    movement = (left_offset + right_offset) .* 0.25 .* obj.linear_vel;
    
                    if vecnorm(movement) < 10*obj.linear_vel
                        graspFlag = -1;
                    end
       
                    positionOut = [positionIn(1:3) + movement, positionIn(4) + rotation ];
                    successFlag = 1;
                else
                    positionOut = positionIn;
                    successFlag = 0;
                end
            else
                positionOut = positionIn;
                successFlag = 0;
            end

        end


        function [obj, positionOut, successFlag, positionCostStruct] = updatePosition(obj, positionIn, qIn)
            %UPDATEPOSITION Get the next position from the queue
            % If the queue is empty and there is a goal, then generate
            % another trajectory to be added to the queue then try again.
            %
            % Function Properties:
            %   obj - POSITIONCONTROLCLASS Class object
            %
            %   positionIn - [x y z yaw] of the ant at the current time step.
            %
            %   qIn - 10x1 vector of the joint angles of the rigidBodyTree
            %   at the current time step.
            %[COST] PositionCost INIT [TODO]
            positionCostStruct = struct.empty();

            if ~isempty(obj.goal)
                if isempty(obj.trajectory_queue)
                    try
                        obj = obj.loadTrajectory(positionIn, qIn);
                    catch
                        warning("Could not load a new trajectory - likely goal is already reached")
                        %obj.goal = goalStruct;
                    end
                end
            end
            if obj.RUNTIME_ARGS.BODY_NAV_MODE == "follow"
                [linVel , angVel] = obj.controller([positionIn(1:2), positionIn(3)]);
                
                nextPosition = [positionIn(1:2) + linVel, positionIn(4) + angVel];
                goalRadius = 0.1;
                distanceToGoal = norm(positionIn - robotGoal);
                if distanceToGoal < goalRadius
                    successFlag = 0;
                else
                    successFlag = 1;
                end
            else
                [obj, nextPosition, successFlag] = tbox.popTrajectory(obj);
            end
            if successFlag
                %Invert position due to dimensions of queue
                positionOut = nextPosition';
            else
                positionOut = positionIn; %If it did not work, just don't move
            end


        end

        function obj = updateGoal(obj, contactStruct, positionIn, goal)
            %UPDATEGOAL Use the contact_points struct from ant memory, the
            %input position and the goal location to assign class
            %properties and define a map for path planning.
            % Function Properties:
            %   obj - POSITIONCONTROLCLASS Class object
            %
            %   contactStruct - Struct of contact points with both "limb"
            % and "point" properties from the ANT class.
            %
            %   positionIn - [x y z yaw] of the ant at the current time step.
            %
            %   goal - an instance of goalStruct containing the position to
            %   grasp and the heading vector

            %Add the environmental collision data to the map of the env
            obj = obj.makeMap(contactStruct, positionIn);

            %Call the body motion controller and update the goal
            obj.goal = goal;
        end

        function obj = makeMap(obj, contactStruct, positionIn)
            %MAKEMAP Define occupancy grid for path planning
            % Uses binaryOccupancyMap to convert contact locations to
            % collision points within a 2D navigation plane.
            % Function Properties:
            %   obj - POSITIONCONTROLCLASS Class object
            %
            %   contactStruct - Struct of contact points with both "limb"
            % and "point" properties from the ANT class.
            %
            %   positionIn - [x y z yaw] of the ant at the current time step.


            obj.map = binaryOccupancyMap(obj.RUNTIME_ARGS.MAP_SIZE.WIDTH, obj.RUNTIME_ARGS.MAP_SIZE.HEIGHT, 50); %Creates a binary occupancy map with 50 squares per meter(per unit square)

            grid_location_in_world = [positionIn(1) - (obj.RUNTIME_ARGS.MAP_SIZE.WIDTH/2), ...
                positionIn(2) - (obj.RUNTIME_ARGS.MAP_SIZE.HEIGHT/2)]; % Places the current ant position as the centre of the map
            obj.map.GridLocationInWorld = grid_location_in_world;


            mat_pts = cat(1,contactStruct(:).point); % matrix of each of the contact points as 1x3 vectors
            occupancy = mat_pts(:, 1:2); % isolate only the x and y values

            setOccupancy(obj.map, occupancy, 1);

            inflate(obj.map, obj.RUNTIME_ARGS.MAP_POINT_RADIUS); % increase the occupancy space of each of the contact points to a set collision radius

        end


        function obj = loadTrajectory(obj, positionIn, qIn)
            %Define the goal [x y z yaw] position including offset from the object and orientation

            mandGoal(1:3) = obj.goal.midpoint;

            mandGoal(4) = tbox.findGlobalAngleOffset(obj.goal.alignment_axis, [0 1 0], [0 0 1]);

            TMand = getTransform(obj.bodyTree, qIn, 'mandible_base_link');
            %TMand = getTransform(obj.bodyTree, homeConfiguration(obj.bodyTree), 'mandible_base_link');

            positionGoal = tbox.offsetPositionByTransform(mandGoal, TMand, positionIn);


            switch obj.RUNTIME_ARGS.BODY_NAV_MODE
                case "align"
                    %Generate a planned A* trajectory between the current
                    %position and an offset position from the contact
                    %midpoint
                    %Find offset location
                    positionOffset = obj.goal.alignment_axis * 2;

                    offsetGoal = positionGoal;
                    offsetGoal(1:3) = offsetGoal(1:3) - positionOffset;

                    macroPathQueue = obj.trajFromAstar(positionIn, offsetGoal);

                    %Add a linear trajectory to the end that makes the ant
                    %head approach along the alignment vector
                    obj.trajectory_queue = [macroPathQueue, ...
                        obj.generateTrapTraj(macroPathQueue(:, end), positionGoal)];


                case "goal"
                    %Generate a straight line trajectory between the current
                    %position and an offset position from the contact
                    %midpoint without any offset
                    obj.trajectory_queue = obj.generateTrapTraj(positionIn, positionGoal);

                case "follow"
                    %Generate a trajectory then follow it separately
                    positionOffset = obj.goal.alignment_axis * 2;

                    offsetGoal = positionGoal;
                    offsetGoal(1:3) = offsetGoal(1:3) - positionOffset;

                    macroPathQueue = obj.trajFromAstar(positionIn, offsetGoal);

                    %Add a linear trajectory to the end that makes the ant
                    %head approach along the alignment vector
                    obj.trajectory_queue = [macroPathQueue, ...
                        obj.generateLineTraj(macroPathQueue(:, end), positionGoal)];

                    release(obj.controller)
                    obj.controller.Waypoints = obj.trajectory_queue([1,2,4],:)';

            end

        end

        function trajectory = generateTrapTraj(obj, positionIn, positionGoal)
            %Generate an interpolated path from the current position to
            %the point where the midpoint of the goal is at the
            %mandible base
            %Asserts the dimensions to be 1x4
            start_pt = [positionIn(1),positionIn(2),positionIn(3), positionIn(4)];
            end_pt = [positionGoal(1),positionGoal(2),positionGoal(3), positionGoal(4)];
            %start_pt = positionIn;
            %end_pt = positionGoal;


            %3x2 Matrix of the start and end points for the trajectory
            wayPoints = [start_pt; end_pt]';



            peakVelocity = [ones([3 1])* obj.RUNTIME_ARGS.BODY_VEL_LIMITS.LINEAR ; ...
                obj.RUNTIME_ARGS.BODY_VEL_LIMITS.ANGULAR]*obj.RUNTIME_ARGS.RATE;
            distance = sqrt(sum((wayPoints(1:3,1)-wayPoints(1:3,2)).^2));

            %Ensure that the number of samples is more than required at
            %half of the max speed
            numSamples = ceil(distance/mean(peakVelocity(1:3)));

            [q,~,~,~,~] = trapveltraj(wayPoints,numSamples, ...
                'PeakVelocity', peakVelocity);


            q(:,1) = [];
            trajectory = q;

        end

        function trajectory = generateLineTraj(~, positionIn, positionGoal)
            %Generate an interpolated path from the current position to
            %the point where the midpoint of the goal is at the
            %mandible base
            start_pt = positionIn(1:3);
            end_pt = positionGoal(1:3);

            %Interpolate waypoints between these two points
            vector = end_pt - start_pt;

            step_size = 0.05; %divide the path in to 20 waypoints

            for i = 1:length(end_pt)
                if vector(i) == 0
                    waypoints(:,i) = ones([(1/step_size)+1, 1]) * start_pt(i);
                else
                    waypoints(:,i) = [start_pt(i) : vector(i)*step_size : end_pt(i)];
                end
            end

            waypoints(1,:) = [];
            trajectory = [waypoints, ones([size(waypoints,1), 1])*positionIn(4)]';
        end

        function trajectory = trajFromAstar(obj, positionIn, positionGoal)

            % Use A* and the occupancy grid to make a path to the goal (2D)
            validator = validatorOccupancyMap();
            validator.Map = obj.map;
            %validator.ValidationDistance = 0.5;


            %planner = plannerHybridAStar(validator, 'ReverseCost', 1, 'InterpolationDistance', 0.5, 'NumMotionPrimitives', 13);
            planner = plannerHybridAStar(validator, 'ReverseCost', 1);
            planner.MotionPrimitiveLength = 0.3;
            %planner.MinTurningRadius = pi/10;
            planner.InterpolationDistance = obj.RUNTIME_ARGS.BODY_VEL_LIMITS.LINEAR*obj.RUNTIME_ARGS.RATE*2;



            startPose = [positionIn(1:2), positionIn(4)];
            goalPose = [positionGoal(1:2), positionGoal(4)];

            path = plan(planner, startPose, goalPose);


            %Interpolate the Z position
            start_z = positionIn(3);
            end_z = positionGoal(3);
            inc_size = (end_z - start_z)/(path.NumStates-1);
            z_values = [start_z: inc_size: end_z];

            trajectory = [path.States(:, 1:2), z_values', path.States(:, 3)]';
            trajectory(:,1) = []; %Remove the initial state as it is a duplicate of the current position
        end

        function waypoints = AStar2DWaypoints(obj, positionIn, positionGoal)
            %ASTAR2DWAYPOINTS Generate [X Y Yaw] waypoints for the ant to
            %follow to the goal.
            % Use A* and the occupancy grid to make a path to the goal (2D)
            validator = validatorOccupancyMap();
            validator.Map = obj.map;


            planner = plannerHybridAStar(validator, 'ReverseCost', 1);
            planner.MotionPrimitiveLength = obj.RUNTIME_ARGS.BODY_VEL_LIMITS.LINEAR*obj.RUNTIME_ARGS.RATE*2;



            startPose = [positionIn(1:2), positionIn(4)];
            goalPose = [positionGoal(1:2), positionGoal(4)];

            path = plan(planner, startPose, goalPose);

            waypoints = path.States;

        end

    end
end

