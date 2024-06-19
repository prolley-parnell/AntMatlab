classdef Limb
    %% LIMB
    % A class containing the relevant information for moving extremities on the ant, such as the antennae or mandibles, where each instance represents a single antenna or mandible jaw

    %%

    properties
        name %Identifying name of the limb
        number %Identifying index of the limb (per type)
        type %Limb type, currently "Antenna" or "Mandible"
        colour %Colour used to plot or visualise contacts made by this limb
        end_effector %String name that is allocated to the rigidBodyTree link at the end of the limb chain
        base_name %String name of the rigidBodyTree link at the base of the kinematic chain making up the limb
        joint_mask %A binary mask where 1 indicates the joints that are relevant to the current Limb of the full rigidBodyTree list of joints
        joint_limits %An array to store the joint maximum and minimum positions, extracted from the rigidBodyTree from the URDF
        idx %Array of integers for the positions of the Limb links in the list of links that make up the full rigidBodyTree
        subtree %a rigidBodyTree that consists only of the kinematic chain for the current Limb
        full_tree %A copy of the full Ant rigidBodyTree
        free_point %A 3x1 array of the last cartesian position of the tip of the limb when the full kinematic chain was not in collision with the environment
        free_pose %A joint position array (with only the joints in this limb) with the last values for when the kinematic chain was not in collision with the environment
        trajectory_queue %An array where each column corresponds to a set of joint positions that make up a full trajectory
        collision_latch %A boolean flag to indicate whether the limb was in collision in the previous time step
        control_type %Taken from the RuntimeArgs.SEARCH.MODE to indicate the control method of antennae
        traj_gen_mode %The index used for sample based path generation
        motion_state %Used mainly with mandibles, -1, 0 or 1 are opening, stationary, closing respectively.

    end

    methods
        function obj = Limb(name, number, type, control_type, colour, end_effector, base_name, antTree, qIn, position, RUNTIME_ARGS)
            %LIMB Construct an instance of this class

            %Copy the constructing variables into the current instance of
            %the class
            obj.name = name;
            obj.number = number;
            obj.type = type;
            obj.colour = colour;
            obj.end_effector = end_effector;
            obj.base_name = base_name;
            obj.full_tree = antTree;
            % Using the full rigidBodyTree, find the sub-rigidBodyTree from
            % the specified starting link to the end of the kinematic
            % chain, as well as the indices of the links that make up the
            % sub-tree
            [obj.idx, obj.subtree] = obj.findTreeIdx(antTree, obj.base_name);
            %Using the sub tree link indices, find the properties of the
            %joints in the kinematic chain
            [obj.joint_mask, obj.joint_limits] = obj.findJointMask(antTree, obj.idx);
            %Initialise variables to be empty, collision free, and
            %stationary
            obj.trajectory_queue = [];
            obj.collision_latch = 0;
            obj.control_type = control_type;
            obj = obj.findTrajGenMode();
            obj.motion_state = 0;

            % Set the transform for the subtree in this current Limb to
            % match the transform reflected by the full rigidBodyTree in
            % the initial global position
            obj = obj.updateTreeTransform(qIn, position);

        end



        function obj = updateTreeTransform(obj, qIn, positionIn)
            %% UPDATETREETRANSFORM Update the transform between the base link of the rigidBodyTree and the base of the kinematic chain to match the position offset of the full Ant
            %Find the transform from the base of the full ant rigidBodyTree to the base
            %of the subtree
            %Input:
            % qIn - The current full joint pose of the Ant rigidBodyTree
            % positionIn - The current position of the Ant in the global
            % frame [x, y, z, yaw] where yaw is the Counter
            % Clockwise rotation about the axis [0 0 1]
            %Output:
            % obj - A copy of the current instance of the Limb class
            %%

            baseTF = getTransform(obj.full_tree, qIn, obj.base_name);
            %Find the position of the ant in transform format
            positionTF = tbox.modelPosition2GlobalTF(positionIn);

            %Find the combined transform from the origin-> full ant base -> subtree base
            globalTF = positionTF * baseTF;


            %Set the transform between the parent of the base of the Limb chain
            % to include the global transform to include full Ant position change
            setFixedTransform(obj.subtree.Bodies{1,1}.Joint, globalTF);

        end





        function qSub = findMaskSubsetQ(obj, qFull)
            %% FINDMASKSUBSETQ Extract the joint values relevant to the current Limb instance
            %Input:
            % obj - The current instance of the Limb class
            % qFull - The 10x1 array of joint positions
            %Output:
            % qSub - an nx1 array where n is the number of variabled joints
            % in the subtree for this Limb
            %%
            qSub = qFull(obj.joint_mask==1);
        end

        function  qFullMasked = applyMask(obj, qFullAnt, qLocal)
            %% APPLYMASK Update only the relevant joint variables to match the desired values and leave the remaining the same
            % Input:
            % obj - The current instance of the Limb class
            % qFullAnt - the previous joint state of the full Ant
            % rigidBodyTree
            % qLocal - either the 10x1 new joint values or nx1 where n is
            % the number of controllable joints for this specific Limb
            % Output:
            % qFullMasked - qFullAnt but with the joints corresponding to
            % those controlled by this specific Limb updated to match those
            % in qLocal
            %%
            %Determine whether the qLocal is 10x1 or nx1
            if size(qLocal,1) ~= size(obj.joint_mask,1)
                %Create a 10x1 array of zeros and fill in the relevant
                %joint values from qLocal
                masked_goal = zeros(size(obj.joint_mask));
                masked_goal(obj.joint_mask==1) = qLocal;
            else
                masked_goal = obj.joint_mask .* qLocal;
            end
            %Modify the joints corresponding to the current limb to match
            %those in the masked goal
            qFullMasked = qFullAnt.* (1-obj.joint_mask) + masked_goal;

        end

        function bodyTree = makeHeadTree(~, tree)
            %% MAKEHEADTREE Remove the antennae from the ant head and return the remaining rigidBodyTree without altering the input tree
            % Input:
            % tree - The full rigidBodyTree including the neck, head,
            % mandibles and antennae
            % Output:
            % bodyTree - The rigidBodyTree that is not linked to the input
            % tree and consists of the neck and head and mandibles.
            %%

            %Create a copy of the full rigidBodyTree as to not make changes
            %to the original
            head_subtree = copy(tree);

            %Remove both antennae chains from this head_subtree (returns
            %itself)
            removeBody(head_subtree, 'left_antenna_base');
            removeBody(head_subtree, 'right_antenna_base');

            bodyTree = head_subtree;

        end

        function subTree = makeSubTree(~, tree, base_name)
            %% MAKESUBTREE Remove all subsequent links after the link determined by base_name and return the remaining tree
            % Does not modify the input tree and instead uses a copy
            % Input:
            % tree - a rigidBodyTree object
            % base_name - a string with a name that matches one of the
            % links found in the rbt tree
            % Output:
            % subTree - a rigidBodyTree object without any links connected
            % after the base_name link

            tree_copy = copy(tree);
            %Checks in case the base_name matches the base of the tree and
            %causes an error
            if ~strcmp(base_name, tree.BaseName)
                subTree = removeBody(tree_copy, base_name);
            else
                subTree = tree_copy;
            end
        end


        function [body_idx, subtree] = findTreeIdx(obj, tree, varargin)
            %% FINDTREEIDX Using the full rigidBodyTree, extract the Limb subtree and the corresponding link indices that map to the full tree
            % Input:
            % tree - a rigidBodyTree object usually of the full Ant model
            % varargin - options
            %   a) a rigidBodyTree object already extracted that is a subtree
            %   of the full tree
            %   b) multiple cells as text descriptors of where to break the
            %   full tree e.g. {'antenna', 'right'} or {'jaw'} - Throws an
            %   error if the side is not specified and the first cell is
            %   not the object type
            %   Will throw an error is the subtree is not in the first
            %   parameter and a base link name is also included
            % Output:
            % body_idx - The integer values that can be used to address the
            % "Bodies" that make up the full rigidBodyTree of the Ant and
            % map to the links that make up the subtree
            % subtree - The rigidBodyTree object made up of the links and
            % joints after the specified base link name.
            %%

            % Rename functions to identify whether the input argument is a
            % valid rigidBodyTree and to detemine whether the input is a
            % valid text format. Used for readability.
            validTree = @(x)isa(x, 'rigidBodyTree');
            validText = @(x) isstring(x) || ischar(x) ;


            %If the user provides only a valid subtree, use this input
            if and(length(varargin) == 1, validTree(varargin{1}))
                subtree = varargin{1};

                % If the first input parameter is a valid text input
            elseif validText(varargin{1})
                % Define the possible components of the text input
                LR = ['left', 'right'];
                AJ = ['antenna', 'jaw'];

                %If the first input argument specifies a limb type (found in AJ), and they provide more than one input
                if and(any(strcmp(varargin{1}, AJ)) , length(varargin) >1)

                    %Extract the specific string in the correct text case
                    part_name = AJ(strcmp(varargin{1}, AJ));
                    side_name = LR(strcmp(varargin{2}, LR));

                    % Compose a string matching the link naming structure
                    new_base_name = side_name + '_' + part_name + '_base';
                    %Crop the subtree to only the side specified
                    subtree = obj.makeSubTree(tree, new_base_name);

                else %If there is more than one input argument or the first parameter is not matching one of the limb types
                    % Assume the first input argiment is a text string that
                    % matches the naming structure of the base link in the
                    % full rigidBodyTree
                    base_link_name = varargin{1};
                    %Extract the subtree after this link
                    subtree = obj.makeSubTree(tree, base_link_name);
                end

            else
                % Error, Idx of subtree cannot be found
                disp("You must provide reference to a subtree to index")
                return
            end

            % Extract the string names of all the bodies that make up the
            % subtree
            link_names = subtree.BodyNames;

            % Initialise the set of body_idx to be nan
            body_idx = nan(1, length(link_names));

            % Loop through all the link names and find the index of the
            % corresponding link in the full rigidBodyTree
            for i = 1:length(link_names)
                name = link_names{i};
                body_idx(i) = find(strcmp(tree.BodyNames, name));
            end
        end

        function [mask, joint_limits] = findJointMask(~, tree, body_idx)
            %% FINDJOINTMASK Return a binary mask that identifies the variable joints from a rigidBodyTree object that connect to the links indexed by body_idx
            % Input:
            % tree - a rigidBodyTree object
            % body_idx - integer array with each value mapping to a link in
            % the input tree object
            % Output:
            % mask - a binary nx1 array where n is the total number of variable
            % joints in "tree", and equal to 1 if the joint is connected to
            % links defined by body_idx and 0 otherwise
            % joint_limits - Extracts the maximum and minimum joint limits
            % for the joints indicated by the mask
            %%

            %Initialise the arrays to be empty
            tree_q = [];
            joint_limits = [];

            % Extract the array of all Bodies found in the input tree
            bodies = tree.Bodies(:);

            % Loop across all Bodies in the array
            for i = 1:length(bodies)

                joint_type = bodies{i}.Joint.Type;
                % If the type of the joint is not 'fixed'
                if ~strcmp(joint_type,'fixed')
                    % Find whether this specific body is requested by the
                    % body_idx
                    index = find(body_idx == i);

                    % If the movable joint is relevant as the index is not
                    % empty
                    if ~isempty(index)
                        % Set the mask = 1
                        tree_q(end+1) = 1;

                        % Extract the joint limits of this specific joint
                        joint_limits(end+1,:) = bodies{i}.Joint.PositionLimits;

                    else
                        % Set the mask = 0
                        tree_q(end+1) = 0;
                    end
                end
            end

            % Ensure the mask matches the array size needed for later use
            mask = tree_q';


        end

        function obj = findTrajGenMode(obj)
            % FINDTRAJGENMODE
            % Establish the method of generating goal properties,
            % Through performing IK on all cartesian waypoints
            % OR
            % Finding an IK solution for the endPt and interpolating between qIn and the IK solution
            %---
            if obj.type ~= "Antenna"
                obj.traj_gen_mode = nan;
            else
                trajOption = {'jointInterp', 'cartesianPath'};
                trajGenFlag = contains(trajOption, obj.control_type);

                %Check for errors in assigning the mode of generating the
                %trajectory to the cartesian point.
                if all(trajGenFlag)
                    warning('Too many options selected, default to jointInterp');
                    trajGenMode = 'jointInterp';
                    trajGenFlag = contains(trajOption, trajGenMode);
                elseif any(trajGenFlag)
                    %trajGenMode = trajOption{trajGenFlag==1}
                else
                    warning('No option selected for traj generation, default to jointInterp')
                    trajGenMode = 'jointInterp';
                    trajGenFlag = contains(trajOption, trajGenMode);
                end
                obj.traj_gen_mode = find(trajGenFlag);
            end
        end
    end
end

