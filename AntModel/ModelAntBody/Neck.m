classdef Neck
    %Neck Containts the joints and apropriate functions to control the neck
    %joints on the ant model

    properties
        end_effector
        base_name
        joint_mask
        joint_limits
        idx
        subtree
        full_tree
        trajectory_queue
    end

    methods
        function obj = Neck(antTree, RUNTIME_ARGS)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.end_effector = 'mandible_base_link';
            obj.base_name = 'base_link';
            obj.full_tree = antTree;
            obj.joint_mask = [1 1 0 0 0 0 0 0 0 0]';

            obj.trajectory_queue = [];
        end

        function  qFullMasked = applyMask(obj, qFullAnt, qLocal)
            if size(qLocal,1) ~= size(obj.joint_mask,1)
                masked_goal = zeros(size(obj.joint_mask));
                masked_goal(obj.joint_mask==1) = qLocal;
            else
                masked_goal = obj.joint_mask .* qLocal;
            end
            qFullMasked = qFullAnt.* (1-obj.joint_mask) + masked_goal;

        end

        function qTrajectory = findIKforNeckPose(obj, waypoints, qIn)
            %% FINDIKFORNECKPOSE [BROKEN] - Not tested yet
            %% JOINTCONFIGFROMCARTESIAN Use either pose or 3D XYZ and find the joint configuration for a set of consecutive points in a trajectory

            %Read the shape of the waypoints to determine whether
            %extracting from cartesian or transform based goals
            arrayShape = size(waypoints);

            %Use the last dimension to find the number of waypoints along the path
            numWaypoints = arrayShape(end);
            %waypoints can be either [x y z] or a 4x4 Transform
            %Dimensions 3xn or 4x4xn
            dim = length(arrayShape);

            %Transforms are concatenated in the 3rd dimension whereas
            %cartesian points are concatenated in the 2nd dimension
            switch dim
                case 3
                    %Then using transforms (pose constraint)
                    tform = waypoints;
                    weights = [1 1 1 0 0 0];

                case 2
                    %then using cartesian goals (position)
                    %Convert the waypoints (n x 3) in to transforms
                    tform = trvec2tform(waypoints');

                    %Find the dimension of length of the waypoints (3 x n)
                    weights = [0 0 0 0.8 0.8 0.8];
            end

            %Create an array of zeros to contain the output trajectory
            qTrajectory = [qIn, zeros([(length(qIn)), numWaypoints])];

            %Define the inverse kinematic solver for the subtree
            ik = inverseKinematics('RigidBodyTree', obj.full_tree);


            for k = 2:numWaypoints+1
                % set the weights for IK to consider the location more than
                % the orientation of the end effector


                % Find the joint configuration to reach the goal pose
                [qTrajectory(:,k),solInfo] = ik(obj.end_effector,tform(:,:,k-1),weights,qTrajectory(:,k-1));

                if isnan(qTrajectory(:,k)) %If the gik finds an invalid solution (due to other limitations like pose), then go no further and use the trajectory generated so far
                    qTrajectory = qTrajectory(:,1:k-1);
                    warning("IK Produced an invalid pose")
                    break
                end

            end
            % Remove the duplicate initial pose from the full trajectory
            qTrajectory(:,1) = [];


        end
    end
end