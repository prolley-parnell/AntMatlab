classdef graspEvaluator % GRASPEVALUATOR 
    % Class used to interact with an instance of the goalStruct class and calculate quality measures stored in graspQuality objects

    properties
        mu %The static coefficient of friction between the ant body and the object being grasped
        m %number of vectors to approximate cone
        force_applied %A scalar in Newtons that is the force applied by a mandible half at the point of contact in a grasp
        qhull_arguments %Which specific modifiers are used in the qhull functions
        mandible_max %The distance between the mandible tips (end_effector links) when the mandible joint values are at a maximum and the tips are the furthest possible distance apart
    end

    methods
        function obj = graspEvaluator(RUNTIME_ARGS, mandible_max)
            %% GRASPEVALUATOR Construct an instance of this class
            %   Input:
            % mandible_max - The calculated maximum possible euclidian
            % distance between the mandible points when they are fully open
            %%
            % Copy across the RA value of friction
            obj.mu = RUNTIME_ARGS.GRASP.OBJ_FRICTION;
            % Default: use 8 vectors to approximate the force friction cone
            obj.m = 8;
            %Copy the RA value of force applied
            obj.force_applied = RUNTIME_ARGS.GRASP.FORCE;
            %Copy the values from the RA for q_hull arguments
            obj.qhull_arguments = RUNTIME_ARGS.GRASP.QUALITY.Q_HULL_ARGS;
            obj.mandible_max = mandible_max;

        end

        function qualityObj = evaluateGoal(obj, goalObj, environmentClass)
            % EVALUATEGOAL
            % Carry out all evaluations on the goalStruct instance
            % Input:
            % obj - current instance of the graspEvaluator class
            % goalObj - A filled instance of the goalStruct class, with the
            % contact points and surface normals included
            % environmentClass - a copy of the CollisionObjects instance
            % used to describe the model collision environment
            % Output:
            % qualityObj - a graspQuality object filled with the quality
            % measures of a grasp defined by the goalObj
            %%

            %Find the distance between the two points and the true Centre
            %of Mass
            idx = 1; %All cells contain the Pose and COM of the full shape
            objPose = environmentClass.objectHandles{idx}.Pose;
            objCOM = environmentClass.COM{idx};
            
            %Find the COM of the shape in the global frame (not the shape
            %frame)
            globalCOM = tbox.local2global(objCOM, objPose);

            %Create an instance of the graspQuality class
            qualityObj = graspQuality();

            %Copy the contact point locations
            A = goalObj.contact_point_array(1,:);
            B = goalObj.contact_point_array(2,:);

            %Calculate the offset from the COM to the grasp contacts axis
            qualityObj.com_offset = obj.axis2COM(A, B, globalCOM);

            %Check that the grasp points are closer together than the
            %maximum mandible reach
            IPD = vecnorm(goalObj.contact_axis,2, 2);
            qualityObj.withinReach(IPD>obj.mandible_max) = 0;

            %Calculate the volume and epsilon for the given goal grasp
            %point locations
            forces = obj.genOpposeForces(A, B);
            if ~any(isnan(forces))

                %Collect all vertices in all STL shapes
                all_handle = cat(1, environmentClass.objectHandles{:});
                vertices = cat(1,all_handle.Vertices);
                
                %Find the measure of force alignment with surface normal
                normAlignArray = tbox.findSurfNormAlign(goalObj.contact_norm, forces);

                %The compared measure is the worst alignment
                qualityObj.normAlign = min(normAlignArray);

                %Calculate volume and epsilon through findWrenchQuality
                [qualityObj.volume, qualityObj.epsilon] = obj.findWrenchQuality(forces, [A;B], vertices, globalCOM, objCOM);
            end


        end


        function distance = axis2COM(~, pointA, pointB, COM)
            %AXIS2COM find the shortest distance between the line
            %connecting the two contact points, and the Centre of Mass of
            %the object
            % Input:
            % pointA, pointB - Two cartesian points (1 x 3) that make up
            % the grasp axis
            % COM - A cartesian point (1 x 3)  to compare with the grasp
            % axis
            % Output:
            % distance - The euclidian distance between the COM and the
            % closest point on the axis connecting pointA and pointB
            %%

            %Find vectors connecting the input points
            A2COM = COM - pointA;
            AB = pointB - pointA;
            
            %Use the normalised cross product to find the magnitude of the
            %combined vectors
            distance = norm(cross(A2COM, AB)) / norm(AB) ;

        end

        function [forces] = genOpposeForces(obj, contactA, contactB)
            %% GENOPPOSEFORCES
            %Generate a pair of forces in the direction of
            %the opposing cartesian point in a grasp
            % Input:
            % obj - the current instance of the graspEvaluator class
            % contactA, contactB - Two cartesian points on the surace of an
            % object, these points make up the grasp of a parallel gripper
            % Output:
            % forces - (2 x 4) where the first three columns are the unit
            % vector directions of force, and the 4th column is the
            % magnitude of force as defined at class instantiation. Row 1
            % is the force applied at contactA, and row 2 is the force
            % applied at contactB

            force_a = contactB - contactA;
            forceA_norm = force_a/vecnorm(force_a);
            forceB_norm = -forceA_norm;

            forceA = [forceA_norm,obj.force_applied];
            forceB = [forceB_norm,obj.force_applied];
            forces = [forceA;forceB];


        end
        function [wrench] = wrench3d(obj, forces, contacts, globalCOM, lamda)
            %WRENCH3D calculate the 6D wrench space generated by n contacts
            %in the global frame
            % Input:
            % obj - current instance of the graspEvaluator class
            % forces - (n x 4) force direction and magnitude applied by the
            %grasp at the contact point in contacts on the same row
            % contacts - (n x 3) Cartesian points in the global frame on
            % the surface of the object being grasped, the force is applied
            % at these points
            % globalCOM - The object's centre of mass in the global frame
            % (1 x 3) cartesian point.
            % lamda - scaling factor to account for the size of the object
            % being grasped
            % Output:
            % wrench - (6 x n*m ) a matrix containing the wrench forces
            % applied by the contacts, made up of the force and torque.
            %%
            
            
            num_contacts = length(contacts(:,1));

            %Initialise the torque and wrench
            torque = zeros(3, num_contacts*obj.m);
            wrench = zeros(6, num_contacts*obj.m);

            for i = 1:num_contacts
                cp_idx = (i - 1)*obj.m; %To fit the force vectors within the matrix

                % Create a set of vectors to approximate a cone
                %If mu > 0, represent forces using friction cone, else, a single vector
                cone = obj.coneFromVec(forces(i,:), obj.mu, obj.m);

                %Find the torque
                for j = 1:obj.m
                    % Torque is the cross product between the vector
                    % between the contanct point and COM and the cone
                    % vectors
                    torque(:, cp_idx+j) = cross(contacts(i,:)- globalCOM, cone(j,:));
                end

                %Convert in to wrench space and scale using lambda
                wrench(1:3, cp_idx+1:cp_idx+obj.m) = cone';
                wrench(4:6, cp_idx+1:cp_idx+obj.m) = torque(:, cp_idx+1:cp_idx+obj.m).*lamda;
            end

            wrench = wrench';

        end

        function [volume, epsilon] = findWrenchQuality(obj, forces, points, vertices, globalCOM, objCOM)

            %Find max object radius for scaling factor

            r = max(vecnorm(vertices,2, 2));
            lamda = 1/r;

            wrench = obj.wrench3d(forces, points, globalCOM, lamda);


            [chull, volume] = convhulln(wrench,obj.qhull_arguments);
            epsilon = obj.estability(objCOM, wrench, chull);

        end



        function [epsilon] = estability(~, point, wrenchT,chull)
            %ESTABILITY Find the largest radius circle that fits within the 6D wrench
            %space
            % If point is within hull, find the minimum distance between point and hull
            if length(point) == 3
                %If the point used is the 3D COM, duplicate it in to 6D
                point = [point,point];
            end

            tol = 1.e-13*mean(abs(wrenchT(:)));
            [~, epsilon] = inhull(point,wrenchT,chull, tol);

            if epsilon<1e-9 %arbitrary threshold
                epsilon = 0; %Wipe out errors in calculations
            end

        end


        function [cone, R] = coneFromVec(~, vector, mu, m)
            %CONEFROMVEC Uses the force vector and generates a force cone based on the
            %coefficient of friction
            %   Plot a circle with radius f*mu
            %   Transpose the circle in the Z axis along the magnitude of the force
            %   vector
            %   Rotate the translated circle about the axes to align [0 0 1] with the
            %   input vector
            %   Return the array of m forces that make up the polyhedral friction cone

            circ = zeros([m, 3]);
            ref_vec = [0 0 vector(4)];

            if mu > 0
                radius = vector(4)*mu; %Cone Radius

                %Circle points
                for j = 1:m
                    alp = 2*pi*j / m;
                    circ(j, 1) = radius * sin(alp);
                    circ(j, 2) = radius * cos(alp);
                    circ(j, 3) = vector(4); %Make height equal to force magnitude
                end


                % Cross product
                A = ref_vec(1:3);
                B = vector(1:3);



                cross_th = asin(sqrt(sum((cross(A,B)).^2)) / (sqrt(sum(A.^2)) * sqrt(sum(B.^2))));

                %Because the starting vector has +ve Z, if the Z of the desired vector
                %is not the same, they must be over 90 degrees apart
                if  sign(B(3)) == -1
                    cross_th = deg2rad(180) - cross_th;
                end

                %[ERROR] cross(B,A) = [0 0 0] - Cannot be used in
                %axang2rotm
                %[Fix]
                %If the cross axis is non-existent because the vector is in line with the Z axis
                if all(~logical(cross(B,A)))
                    rotAxis = [1 0 0]; %Give another standard axis of rotation (perpendicular to Z axis)
                else
                    rotAxis = cross(B,A);
                end

                R = axang2rotm([rotAxis cross_th]);
                cone = circ * R;
                v = sum(cone,1)/8;

                % Remove any rounding errors
                tolflag = abs(v) < 10e-9;
                v(tolflag) = 0;


                % Cross product gives the acute angle but not necessarily the sign, so
                % it could be aligned with the -ve cross product. This fixes this
                if any(sign(v) ~= sign(vector(1:3)))
                    cone = -cone;
                end

            else
                cone = vector(1:3);
            end

        end


    end
end

