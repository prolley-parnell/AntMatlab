classdef tbox
    %tbox A class containing functions used in transformations
    %between different reference frames, and common model memory
    %manipulations.
    %   ChangeLog: 12/09/22 - Persie Rolley-Parnell - resolvedissues with
    %   global2rotmat by ensuring that all the angles are calculated
    %   against references consistently

    methods (Static)
        function [trajectory, velocity] = trapVelGen(interval, waypoints, velLimits)
            %% JOINTTRAJECTORY Find intermediate joint positions between the waypoint joint values
            % Input:
            % obj - current instance of the SampleActionGen class
            % waypoints - [n x m] - n joint values for the subtree
            % evaluated across m waypoints
            % velLimits - [n x 1] - n joint velocity limits corresponding to the n joints in the sub-rigidBodyTree
            % Output:
            % qTrajectory - [n x i] - joint trajectory across i steps -
            % length varies based on velocity limits
            %%

            %Check whether the start and end position are different
            nJoint = size(waypoints,1);
            trajectoryCell = cell([nJoint,1]);
            velocityCell = cell([nJoint,1]);

            roundWP = round(waypoints,2);
            
            [~, uniqueWPi] = unique(roundWP', 'stable', 'rows');
            waypoints = waypoints(:, uniqueWPi);

            if size(waypoints, 2) < 2
                trajectory = [];
                velocity = [];
            else
                nJointAngle = nan(nJoint,1);
                 % Initially divide the trajectory into 500 samples
                numSample = 500;

                for n=1:nJoint
                    if ~isequal(waypoints(n,1), waypoints(n,2))
                        %Calculate the joint values q and the corresponding time t that
                        %adheres to velocity limits using the trapezoidal velocity
                        %limits
                        [q, qd, ~, t] = trapveltraj(waypoints(n,:),numSample,...
                            PeakVelocity=velLimits(n));

                        % Scale down the joint times to match the sampling rate (vel
                        % limits is in rads/s)
                        % This makes the joint times into multiples of the sampling
                        % rate
                        div_t = t / interval;

                        % Round the multiples of joint times to their nearest integers
                        round_div_t = round(div_t);
                        
                        % Downsample the joint times to find the index of each first multiple of the time step
                        [~, i_first_unique] = unique(round_div_t, 'stable');
                        
                        % Extract the joint position and velocity at these downsampled times
                        trajectoryCell{n} = [q(i_first_unique), q(end)];
                        velocityCell{n} = [qd(i_first_unique), qd(end)];
                        
                        %Find the number of steps required to reach the
                        %goal angle
                        nJointAngle(n) = length(trajectoryCell{n});
                    end
                end
                
                %Identify the longest duration
                trajLength = max(nJointAngle);
                %Extend the trajectory to be as long as the longest
                %duration
                trajectory = ones([nJoint, trajLength]).*waypoints(:,end);

                velocity = zeros(size(trajectory));
                for m=1:nJoint
                    if ~isempty(trajectoryCell{m})
                        trajectory(m,[1:nJointAngle(m)]) = trajectoryCell{m};
                        velocity(m,[1:nJointAngle(m)]) = velocityCell{m};
                    end
                end
            end
        end

        function globalPointArray = local2global(localPointArray, localFrameTF)
            %LOCAL2GLOBAL Transform an array of cartesian 3D points from a
            % local reference frame, defined by localTF, to the global frame.
            %Inputs:
            %   localPointArray - nx3 array of cartesian points in a local
            %   frame.
            %   localFrameTF - 4x4 Heterogeneous Transform from the global frame
            %   to the origin of the reference frame used by the
            %   localPointArray.
            % Outputs:
            %   globalPointArray: nx3 array of cartesian points in the
            %   global reference frame.


            %local to global -> rotate then translate
            globalPointArray = (tform2rotm(localFrameTF) * localPointArray')'...
                + tform2trvec(localFrameTF);

        end

        function localPointArray = global2local(globalPointArray, localFrameTF)
            %LOCAL2GLOBAL Transform an array of cartesian 3D points from a
            % local reference frame, defined by localTF, to the global frame.
            %Inputs:
            %   globalPointArray - nx3 array of cartesian points in the
            %   global reference frame.
            %   localFrameTF - 4x4 Heterogeneous Transform from the global
            %   frame origin to the origin of the new frame of reference that will be
            %   used in the localPointArray
            %
            % Outputs:
            %   localPointArray: nx3 array of cartesian points in a
            %   local reference frame defined by localFrameTF.

            %global to local ->  translate then rotate
            localPointArray = tform2rotm(localFrameTF) * (globalPointArray - tform2trvec(localFrameTF)');

        end

        function globalModelTF = modelPosition2GlobalTF(positionIn)
            %modelPosition2GlobalTF Uses the position required to plot a
            %RigidBodyTree and converts it in to a Homogeneous Transform.
            % Inputs:
            %   positionIn: 1x4 vector representing the position of the ant
            %   in global space. [X, Y, Z, Yaw]. Yaw is the Counter
            %   Clockwise rotation about the axis [0 0 1]. This positionIn
            %   refers to the origin of the base_link of the
            %   RigidBodyTree.
            % Outputs:
            %   globalModelTF: positionIn in the form of a 4x4 homogeneous
            %   transform

            translation = positionIn(1:3);

            %RBT PositionIn.Yaw is CW at the base up along the Z axis
            rotation = axang2rotm([0 0 1 positionIn(4)]);
            globalModelTF = [rotation, translation' ; 0 0 0 1];
        end


        function [outputObject, outputValue, successFlag] = popTrajectory(inputObject)

            outputObject = inputObject;

            try
                outputValue = outputObject.trajectory_queue(:,1);
                outputObject.trajectory_queue(:,1) = [];
                successFlag = 1;
            catch
                outputValue = nan;
                successFlag = 0;
                if ~isempty(inputObject.trajectory_queue)
                    warning("Could Not Pop Trajectory of %s", class(inputObject))
                end

            end

        end


        function waypoints = generateNeckTrajectory(neckIn, qIn, goalStructIn)
            %% GENERATENECKTRAJECTORY For rotating the head of the model using the neck joints
            % Input:
            % obj - current instance of the SampleActionGen class
            % neckIn - an instance of the Neck class
            % qIn - The full joint configuration of the ant rigidBodyTree
            % goalStructIn - a filled instance of the goalStruct class
            % Output:
            % waypoints - 4x4xn array of transformations mapping to each intermediate pose towards the goal 
            %%

            % Find the rotation transform required to make the ant head
            % align with the desired grasp
            [~,global2goalR] = tbox.findGoalrotmat(goalStructIn);

            % Extract the start and end links in the kinematic chain
            sourcebody = neckIn.base_name;
            targetbody = neckIn.end_effector;

            % Find the transform between the neck end and base (reverse) within the
            % full ant model tree at the current pose
            currentPoseTF = getTransform(neckIn.full_tree, qIn, targetbody, sourcebody);
            % Find the transform between the neck end and base (reverse) with the
            % full ant model but at the default home pose
            base2EETF = getTransform(neckIn.full_tree, homeConfiguration(neckIn.full_tree), targetbody, sourcebody);

            % Transform from the neutral pose, not the current pose to find
            % the goal head rotation
            goalPose = rotm2tform(global2goalR) * base2EETF;


            % interpolate between the current pose and the rotated pose
            tSamples = 0:0.05:1;
            [waypoints,~,~] = transformtraj(currentPoseTF,goalPose,[0 1],tSamples);


            
        end



        function [mandibleOut, successFlag] = loadMandibleTrajectory(mandibleIn, qIn, maxVelocity, interval)
            %% LOADMANDIBLETRAJECTORY Generate a joint trajectory to follow the path of motion set in a mandible Limb instance
            % Input:
            % obj - current instance of the SampleActionGen class
            % mandibleIn - An instance of the Limb class corresponding to
            % a mandibular jaw
            % qIn - the current full body joint configuration
            % maxVelocity - The velocity limits of the full ant
            % rigidBodyTree
            % interval - The time step between each cycle of the simulation
            % (RA.RATE)
            % Output:
            % mandibleOut - The modified instance of the Limb class relating to the mandible
            % successFlag - Boolean flag to indicate whether a trajectory
            % was successfully added to the Limb instance
            %%
            % Initialise the mandibleOut to be a copy of mandibleIn
            mandibleOut = mandibleIn;

            try
                if and(abs(mandibleIn.motion_state), ~mandibleIn.collision_latch) %If the mandible is opening or closing and is not in collision currently
                    if mandibleIn.motion_state < 0
                        %opening
                        col = 2;
                    else
                        %closing
                        col = 1;
                    end

                    %Find the joint limits for the mandibleIn
                    goal_joint_val = mandibleIn.joint_limits(:,col);
                    %Find the current pose of the mandible subtree
                    current_joint_val = qIn(mandibleIn.joint_mask==1);

                    %Find the start and end joint pose values to
                    %interpolate between
                    points = [current_joint_val , goal_joint_val];

                    %Divide the trajectory into 10 intervals
                    numSamples = 10;

                    %Find the joint values to move between the current and
                    %the desired mandible pose
                    [q, ~, ~, ~, ~] = trapveltraj(points, numSamples, 'PeakVelocity', maxVelocity(mandibleIn.joint_mask==1)*interval);
                    %Remove the duplicate position
                    q(:,1) = [];

                    %Save the joint trajectory to the mandible Limb object
                    mandibleOut.trajectory_queue = q;
                else
                    mandibleOut.trajectory_queue = [];
                end
                successFlag = 1;
            catch
                successFlag = 0;
            end
        end

        function goalOut = offsetPositionByTransform(goalIn, tformIn, positionIn)
            %OFFSETPOSITIONBYTRANSFORM Given a goal position to navigate to,
            %change that position so that it is reached by a given
            %rigidBodyTree link rather than the base of the robot
            %Input:
            %  TFORM must be the transform from the base link to the new
            %  reference link
            %  positionIn is the desired [X Y Z CCWYaw] of the overall ant head
            %  alignment


            offset = tform2trvec(tformIn)';
            rotatedOffset = tform2rotm(tbox.modelPosition2GlobalTF(goalIn)) * offset;
            goalOut = goalIn;
            goalOut(1:3) = goalIn(1:3) - rotatedOffset';


        end

        function globalPosition = findFKglobalPosition(RBTree, qIn, positionIn, endEffectorName)
            %FINDENDEFFECTORGLOBALPOSITION Find the cartesian coordinates of an
            %end effector
            % Given the pose of the model, the position in the global frame of
            % the base link of the rigid body tree, and the name of the end
            % effector.

            base2EETF = getTransform(RBTree, qIn, endEffectorName);

            global2baseTF = tbox.modelPosition2GlobalTF(positionIn);

            localEndPosition = tform2trvec(base2EETF);
            globalPosition = tbox.local2global(localEndPosition, global2baseTF);


        end

        function localPosition = findFKlocalPosition(RBsubTree,qIn,endEffectorName)

            base2EETF = getTransform(RBsubTree, qIn, endEffectorName);
            localPosition = tform2trvec(base2EETF);

        end

        function cwAngle = findGlobalAngleOffset(goalVector, referenceAxis, rotationAxis)
            %FINDGLOBALANGLEOFFSET Find the clockwise angle of rotation about a rotationAxis
            %from a given referenceAxis TO the goalVector
            %ReferenceAxis and rotationAxis must be unit in one of the
            %cartesian frames (X, Y or Z)
            % Rotation is calculated with the reference being 12 on the
            % clock and is calculated from an egocentric (from the origin
            % looking out along the axis of rotation)

            rotationAxis_n = rotationAxis/norm(rotationAxis);

            %Mask the comparison vectors according to the axis of rotation
            referenceAxis(rotationAxis_n == 1) = 0;
            goalVector(rotationAxis_n == 1) = 0;


            %Normalise each vector that has been masked
            referenceAxis_m_n = referenceAxis/norm(referenceAxis);
            goalVector_m_n = goalVector/norm(goalVector);


            %Both vectors must be column vectors
            dotProduct = dot(referenceAxis_m_n', goalVector_m_n');
            acuteAngle = acos(dotProduct);

            %Identify how to process the output angle, given the axis of
            %rotation and the goal

            dimension = find(rotationAxis_n == 1);
            %Using left hand rule, if the rotation is about Y axis, then
            %results change
            invertB = 1;
            if dimension == 1
                idx_a = 3;
                idx_b = 2;
                invertB = -1;

            elseif dimension == 2
                idx_a = 3;
                idx_b = 1;

            elseif dimension == 3
                idx_a = 2;
                idx_b = 1;
                invertB = -1;

            else
                warning("Something is incorrect, please select an axis of rotation that is unit in  X,Y or Z global frame")
                idx_a = 0;
                idx_b = 0;

            end


            if and(sign(goalVector_m_n(idx_a)) ~= sign(referenceAxis_m_n(idx_a)), acuteAngle <= pi/2)
                acuteAngle = pi - acuteAngle;
            end
            cwAngle = acuteAngle * invertB * sign(goalVector_m_n(idx_b));

        end


        function [goalYawAngle, headPoseRMat] = findGoalrotmat(goalStructIn)
            %FINDGOALrotmat Give the Yaw of the goal position and the
            %approximate roll and pitch required to align the head
            goalAlignAxis = goalStructIn.alignment_axis;
            goalZAxis = goalStructIn.goal_z_axis;

            %Yaw
            referenceAxisYaw = [0 1 0];
            rotationAxisYaw = [0 0 1];
            goalYawAngle = tbox.findGlobalAngleOffset(goalAlignAxis, referenceAxisYaw, rotationAxisYaw);

            %Rotate the goalZ axis to convert it in to the frame of the
            %alignment axis
            goalZAxis_align_y = [axang2rotm([rotationAxisYaw, -goalYawAngle]) * goalZAxis']';
            goalZAxis_align_y_n = goalZAxis_align_y / norm(goalZAxis_align_y);

            %Pitch
            referenceAxisPitch = [0 0 1];

            rotationAxisPitch = [1 0 0];
            pitchAngle = tbox.findGlobalAngleOffset(goalZAxis_align_y_n, referenceAxisPitch, rotationAxisPitch);

            goalZAxis_align_yp = [axang2rotm([rotationAxisPitch, -pitchAngle]) * goalZAxis_align_y_n']';
            goalZAxis_align_yp_n = goalZAxis_align_yp / norm(goalZAxis_align_yp);

            %Roll
            referenceAxisRoll = [0 0 1];
            rotationAxisRoll = [0 1 0];

            rollAngle = tbox.findGlobalAngleOffset(goalZAxis_align_yp_n, referenceAxisRoll, rotationAxisRoll);

            rollR = axang2rotm([rotationAxisRoll rollAngle]);
            pitchR = axang2rotm([rotationAxisPitch pitchAngle]);

            headPoseRMat = pitchR * rollR;
            %[TODO] Convert angles to euler Z Y X (default) eul2rotm([X Y Z])


        end
        function [pointOnObj, normalVArray, vertIDArray, faceIDArray] = findPointOnObjNormalID(fbTriang, samplePoint)
            %findNormalCollision for a given contact point on a
            %delaunayTriangulation object, and the object, find the normal
            %to the surface at the point of contact.
            %Input:
            % fbTriang - triangulation object for the item with which the
            % collision occured
            % pointOnObj -  nx3 cartesian where n is number of sample points
            %Output: 
            % pointOnObj - The Cartesian point on the object, different
            % from sample point if rounding leads to slight error
            % normalVArray - The surface normal at the point pointOnObj
            % vertIDArray - the vertext ID in the triangulation of the
            % pointOnObject if the collision occured at a vertex
            % faceIDArray - the face ID in the triangulation object if the
            % collision occured on a face plane


            nPoint = size(samplePoint,1);
            pointOnObj = nan(size(samplePoint));
            normalVArray = nan(size(samplePoint));
            vertIDArray = nan([nPoint,1]);
            faceIDArray = nan([nPoint,1]);

            for n = 1:nPoint
                nearVertID = nearestNeighbor(fbTriang, samplePoint(n,:));
                nearV_cartesian = fbTriang.Points(nearVertID, :);
                nearV_dist = vecnorm(nearV_cartesian - samplePoint(n,:), 2, 2);

                if round(nearV_dist, 8) == 0
                    %The point of contact is on the vertex
                    normalVArray(n,:) = vertexNormal(fbTriang, nearVertID);
                    pointOnObj(n,:) = fbTriang.Points(nearVertID,:);
                    vertIDArray(n,:) = nearVertID;
                else

                    nearV_neighbours = vertexAttachments(fbTriang,nearVertID);
                    repTestPt = repmat(samplePoint(n,:), length(nearV_neighbours{:}'),1);
                    B = cartesianToBarycentric(fbTriang, nearV_neighbours{:}', repTestPt);
                    % resolves issues with values that are 0 being marked at -0
                    % (9 sig.fig.)
                    roundB = round(B, 9);

                    %Convert back to cartesian coordinates
                    CB = barycentricToCartesian(fbTriang, nearV_neighbours{:}', roundB);
                    CBDist = vecnorm(CB - samplePoint(n,:) ,2,2);
                    [zero_id] = find(~round(CBDist, 6)); %Find the points where the BC matches C coords

                    if length(zero_id)>1
                        pointOnObj(n,:) = CB(zero_id(1), :);
                        faceIDArray(n,:) = nearV_neighbours{:}(zero_id(1));
                        multiNorm = faceNormal(fbTriang, nearV_neighbours{:}(zero_id)');
                        normalVArray(n,:) = mean(multiNorm);
                        


                    elseif length(zero_id) == 1
                        pointOnObj(n,:) = CB(zero_id, :);
                        faceIDArray(n,:) = nearV_neighbours{:}(zero_id);
                        normalVArray(n,:) = faceNormal(fbTriang, faceIDArray(n,:));

                    else
                        %length(zero_id) == 0, find the closest
                        [~, nearestID] = min(CBDist);
                        pointOnObj(n,:) = CB(nearestID,:);
                        faceIDArray(n,:) = nearV_neighbours{:}(nearestID);
                        normalVArray(n,:) = faceNormal(fbTriang, faceIDArray(n,:));
                        

                    end

                end


            end

        end


        function alignMeasure = findSurfNormAlign(surfaceNormal, forceVector)
            %findSurfNormAlign, gives the proportion of the normal vector at
            %the point of contact that is contained within the force applied at
            %that contact point, can check for multiple normal/force pairs
            %Input: surfaceNormal nx3 unit vector for the normal at point of
            %contact
            % forceVector nx4 non unit vector with (i,1:3) containing the
            % direction of force applied, and (i,4) containing the magnitude of
            % force
            nForce = size(forceVector,1);
            nNorm = size(surfaceNormal, 1);
            alignMeasure = nan([nNorm,1]);
            if nForce ~= nNorm
                warning('Must provide the same number of normals as force vectors');
                return
            end

            for n = 1 : nNorm
                %ensure both surf norm and force are unit vectors
                forceVec_n = -forceVector(n, 1:3)/norm(forceVector(n, 1:3));
                normVec_n = surfaceNormal(n,:) / norm(surfaceNormal(n,:));
                alignMeasure(n) = dot(forceVec_n, normVec_n) / norm(forceVec_n);

            end
        end

        function plotRange(limit)

            vertices = nan(8,3);
            %Find the vertices from the range
            %x min
            vertices(1:4,1) = limit(1,1);
            vertices(5:8,1) = limit(1,2);
            vertices([1,2,7,8],2) = limit(2,1);
            vertices(3:6,2) = limit(2,2);
            vertices([1,4,5,8],3) = limit(3,1);
            vertices([2,3,6,7],3) = limit(3,2);

            faces = [1,2,3,4;5,6,7,8;1,2,7,8;3,4,5,6;1,4,5,8;2,3,6,7];

            figure(1)
            patch('Faces', faces, 'Vertices', vertices, 'FaceColor', '#94a088', 'FaceAlpha', 0.1)

        end

        function limitedValues = applyUpperandLowerLimit(inputValue, limitMat)
            %APPLYUPPERANDLOWERLIMIT Input is an mx1 mat and limit is an
            %mx2 mat with lower limit in the first column, and upper limit
            %on the right column
            for m = 1:size(inputValue, 2)
                value_upper_cap = min(inputValue(:,m), limitMat(:,2));
                limitedValues(:,m) = max(value_upper_cap, limitMat(:,1));
            end
        end

    end
end