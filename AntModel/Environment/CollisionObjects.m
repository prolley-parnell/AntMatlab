classdef CollisionObjects
    %COLLISIONOBJECTS
    %Generates and contains the handles to all of the objects within the
    %world
    % REQUIREMENTS: Matlab's Partial Differential Equation Toolbox
    % Changelog - Emily Rolley-Parnell 31/10/2022 - Adding the capability
    % to import multiple STL

    properties
        objectHandles
        DT
        FBT
        COM
        RUNTIME_ARGS

    end

    methods
        function obj = CollisionObjects(RUNTIME_ARGS)
            %COLLISIONOBJECTS Construct an instance of this class
            %  add all of the desired collision objects then show them
            obj.objectHandles = {};
            obj.DT = {};
            obj.FBT = {};
            obj.COM = {};
            obj = obj.addMultiSTLorEmpty(RUNTIME_ARGS.COLLISION_OBJ);
            obj.RUNTIME_ARGS = RUNTIME_ARGS;

            obj = obj.plotObjects();
        end

        function obj = plotObjects(obj)
            if obj.RUNTIME_ARGS.PLOT.ENABLE(1)
                for i= 1:length(obj.objectHandles)
                    figure(1)
                    hold on;
                    [~, patch] = show(obj.objectHandles{i});
                    patch.FaceAlpha = 0.5;
                    patch.EdgeAlpha = 0.7;
                    hold off;
                end
            end
            if obj.RUNTIME_ARGS.PLOT.ENABLE(2)
                for i= 1:length(obj.objectHandles)
                    figure(2)
                    hold on

                    global_COM = tbox.local2global(obj.COM{i}, obj.objectHandles{i}.Pose);
                    plot3(global_COM(1), global_COM(2), global_COM(3), 'x','MarkerSize', 8 , 'Color', 'w', 'LineWidth', 2)

                    hold off
                end

            end
        end



        function obj = addStl(obj, ARGS)

            %Import mesh from file
            meshObj = importGeometry(ARGS.FILE_PATH);
            %Scale mesh according to RUNTIME_ARGS
            scaled_mesh = scale(meshObj, ones([1,3])*ARGS.SCALE);


            %Find the centre point offset of the mesh
            centre_pt = mean(scaled_mesh.Vertices,1);
            %Use transforms in case rotation is used too
            t_zero = trvec2tform(-centre_pt);
            %Zero the vertices so they are all about [0 0 0]
            scaled_zero_mesh = translate(scaled_mesh, tform2trvec(t_zero));


            %Add CollisionBody for mesh
            collision_mesh = collisionMesh(scaled_zero_mesh.Vertices);
            collision_mesh.Pose = trvec2tform(ARGS.POSITION);


            obj.objectHandles{end+1} = collision_mesh;

            obj.COM{end+1} = mean(collision_mesh.Vertices, 1);

            %Calculate surface normals with face indices

            %Add DiscreteGeometry object for mesh
            translate(scaled_zero_mesh, ARGS.POSITION);
            obj.discreteGeomHandle{end+1} = scaled_zero_mesh;


            %Calculate the surface normals for each face
            [obj.DT{end+1}, obj.FBT{end+1}] = obj.collisionToDelaunay(scaled_zero_mesh);



        end

        function obj = addMultiSTLorEmpty(obj, ARGS)
            %ADDMULTISTLOREMPTY Add multiple STLs from a folder path, but
            %updated to include the ability to have empty paths

            %Check whether a mat file has already been generated for this
            %mesh

            
            if isfolder(ARGS.FILE_PATH)
                matFileName = [ARGS.FILE_PATH, '\meshMatSave.mat'];
                matStruct = dir(matFileName);
                if ~isempty(matStruct)
                    loadMat = load(matFileName);
                    obj.objectHandles = loadMat.objectHandles_save;
                    obj.DT = loadMat.DT_save;
                    obj.FBT = loadMat.FBT_save;
                    obj.COM = loadMat.COM_save;
                    return
                end
                folderStruct = dir([ARGS.FILE_PATH, '\*.stl']);
            else
                folderStruct = dir(ARGS.FILE_PATH);
                if isempty(folderStruct)
                    return
                else
                    fileName = split(folderStruct.name, '.');
                    matFileName = [folderStruct(1).folder, '\', fileName{1},'_meshMatSave.mat'];
                    matStruct = dir(matFileName);
                    if ~isempty(matStruct)
                        loadMat = load(matFileName);
                        obj.objectHandles = loadMat.objectHandles_save;
                        obj.DT = loadMat.DT_save;
                        obj.FBT = loadMat.FBT_save;
                        obj.COM = loadMat.COM_save;
                        return
                    end
                end
            end
            allVerticesArray = [];
            nSTL = length(folderStruct);

            model = cell([1, nSTL]);
            for i = 1:nSTL
                %initialise a PDE object
                model{i} = createpde();
                %Import mesh from file
                importGeometry(model{i}, [folderStruct(i).folder,'\',folderStruct(i).name]);
                %Scale mesh according to RUNTIME_ARGS
                scale(model{i}.Geometry, ones([1,3])*ARGS.SCALE);
                scaledMesh = generateMesh(model{i}, GeometricOrder="linear");
                allVerticesArray = cat(1,allVerticesArray,scaledMesh.Nodes');
            end

            CentreOfMass = mean(allVerticesArray,1);

            for j=1:nSTL


                %Translate all vertices so the total mean COM is at 0 0 0
                zeroMesh = model{j}.Mesh.Nodes' - CentreOfMass;

                k = convhulln(zeroMesh);
                %Need to remove indices not referenced by convhill
                newVertIdx = unique(k);
                newVertexArray = zeroMesh(newVertIdx,:);


                %Store the Centre of mass for every STL as the full object COM
                %Set as [0 0 0] as the shapes are already offset by the
                %mean centrepoint of the entire shape - not necessarily the
                %same as the centrepoint of the subshape
                obj.COM{j} = [0 0 0];

                %Convert the translated meshes to CollisionObjects
                collision_mesh = collisionMesh(newVertexArray);
                collision_mesh.Pose = trvec2tform(ARGS.POSITION);

                obj.objectHandles{j} = collision_mesh;

                %Translate to the object pose for the DT calculations
                translateNode = newVertexArray + ARGS.POSITION;

                %Calculate the surface normals for each face
                [obj.DT{j}, obj.FBT{j}] = obj.collisionToDelaunay(translateNode);



            end
            %Save the generated meshes to be loaded later
            objectHandles_save = obj.objectHandles;
            DT_save = obj.DT;
            FBT_save = obj.FBT;
            COM_save = obj.COM;
            save(matFileName, 'objectHandles_save', 'DT_save', 'FBT_save', 'COM_save');

        end
        function obj = addMultiStl(obj, ARGS)
            %ADDMULTISTL Add multiple STLs from a folder path

            %Check whether a mat file has already been generated for this
            %mesh

            if isfolder(ARGS.FILE_PATH)
                matFileName = [ARGS.FILE_PATH, '\meshMatSave.mat'];
                matStruct = dir(matFileName);
                if ~isempty(matStruct)
                    loadMat = load(matFileName);
                    obj.objectHandles = loadMat.objectHandles_save;
                    obj.DT = loadMat.DT_save;
                    obj.FBT = loadMat.FBT_save;
                    obj.COM = loadMat.COM_save;
                    return
                end
                folderStruct = dir([ARGS.FILE_PATH, '\*.stl']);
            else
                folderStruct = dir(ARGS.FILE_PATH);
                fileName = split(folderStruct.name, '.');
                matFileName = [folderStruct(1).folder, '\', fileName{1},'_meshMatSave.mat'];
                matStruct = dir(matFileName);
                if ~isempty(matStruct)
                    loadMat = load(matFileName);
                    obj.objectHandles = loadMat.objectHandles_save;
                    obj.DT = loadMat.DT_save;
                    obj.FBT = loadMat.FBT_save;
                    obj.COM = loadMat.COM_save;
                    return
                end
            end
            allVerticesArray = [];
            nSTL = length(folderStruct);

            model = cell([1, nSTL]);
            for i = 1:nSTL
                %initialise a PDE object
                model{i} = createpde();
                %Import mesh from file
                importGeometry(model{i}, [folderStruct(i).folder,'\',folderStruct(i).name]);
                %Scale mesh according to RUNTIME_ARGS
                scale(model{i}.Geometry, ones([1,3])*ARGS.SCALE);
                scaledMesh = generateMesh(model{i}, GeometricOrder="linear");
                allVerticesArray = cat(1,allVerticesArray,scaledMesh.Nodes');
            end

            CentreOfMass = mean(allVerticesArray,1);

            for j=1:nSTL


                %Translate all vertices so the total mean COM is at 0 0 0
                zeroMesh = model{j}.Mesh.Nodes' - CentreOfMass;

                k = convhulln(zeroMesh);
                %Need to remove indices not referenced by convhill
                newVertIdx = unique(k);
                newVertexArray = zeroMesh(newVertIdx,:);


                %Store the Centre of mass for every STL as the full object COM
                %Set as [0 0 0] as the shapes are already offset by the
                %mean centrepoint of the entire shape - not necessarily the
                %same as the centrepoint of the subshape
                obj.COM{j} = [0 0 0];

                %Convert the translated meshes to CollisionObjects
                collision_mesh = collisionMesh(newVertexArray);
                collision_mesh.Pose = trvec2tform(ARGS.POSITION);

                obj.objectHandles{j} = collision_mesh;

                %Translate to the object pose for the DT calculations
                translateNode = newVertexArray + ARGS.POSITION;

                %Calculate the surface normals for each face
                [obj.DT{j}, obj.FBT{j}] = obj.collisionToDelaunay(translateNode);



            end
            %Save the generated meshes to be loaded later
            objectHandles_save = obj.objectHandles;
            DT_save = obj.DT;
            FBT_save = obj.FBT;
            COM_save = obj.COM;
            save(matFileName, 'objectHandles_save', 'DT_save', 'FBT_save', 'COM_save');

        end

        function [DT, FBT] = collisionToDelaunay(~, vertices)
            %% Generate Delaunay Triangulation representation for
            % surface normal vectors

            x = vertices(:,1);
            y = vertices(:,2);
            z = vertices(:,3);

            DT = delaunayTriangulation(x,y,z);
            [T,Xb] = freeBoundary(DT);
            FBT = triangulation(T,Xb);

            %                         %Plotting
            %             P = incenter(FBT);
            %             F = faceNormal(FBT);
            %             trisurf(T,Xb(:,1),Xb(:,2),Xb(:,3), ...
            %                 'FaceColor','cyan','FaceAlpha',0.8);
            %             axis equal
            %             hold on
            %             quiver3(P(:,1),P(:,2),P(:,3), ...
            %                 F(:,1),F(:,2),F(:,3),0.5,'color','r');









        end

        function [triangTetrIdx, triangulationObj, DTsurfaceNormal] = delaunayToTriang(~, delaunayObj)
            %delaunayToTriang - given a tetrahedal connectivity list, and a
            %free boundary from triangulation, find which triangulations
            %match the tetrahedra

            [T,Xb] = freeBoundary(delaunayObj);
            triangulationObj = triangulation(T,Xb);
            triangConnection = triangulationObj.ConnectivityList;
            delaunaytetra = delaunayObj.ConnectivityList;

            % Split the tetrahedrons in to triangles
            tetLabel = [1:size(delaunaytetra,1)]';
            tetMix = [tetLabel, delaunaytetra];
            facets = [tetMix(:,[1 2 3 4]);tetMix(:,[1 2 3 5]);tetMix(:,[1 2 4 5]);tetMix(:,[1 3 4 5])];
            facets(:,2:4) = sort(facets(:,2:4),2);
            facets = sortrows(facets,[2,3,4]);
            duploc = find(all(diff(facets(:,2:4),1) == 0,2));
            facets([duploc;duploc + 1],:) = [];


            %Find the surface normal of the triangulation
            F = faceNormal(triangulationObj);

            %Find the surface normal of the tetrahedra
            %Find which labelled triangle facet matches the DT triang
            %(same numbers, just different order)
            orderTriang = sort(triangConnection,2);
            [~, facetIdx] = ismember(orderTriang, facets(:,2:4), "rows");
            triangTetrIdx = facets(facetIdx,1);

            matchedTriangNorm = [triangTetrIdx, F];
            sortedTriangNorm = sortrows(matchedTriangNorm);

            %Find the average
            DTsurfaceNormal = 1;

            %Plotting
            P = incenter(triangulationObj);
            trisurf(T,Xb(:,1),Xb(:,2),Xb(:,3), ...
                'FaceColor','cyan','FaceAlpha',0.8);
            axis equal
            hold on
            quiver3(P(:,1),P(:,2),P(:,3), ...
                F(:,1),F(:,2),F(:,3),0.5,'color','r');


        end

        function [triangOrder,nnew] = sortFaces(~, triangArray, vertexArray , normalArray)
            triangOrder = triangArray;
            nnew= normalArray*0;
            for j=1:size(triangArray,1)
                v1 = vertexArray(triangArray(j,3),:)- vertexArray(triangArray(j,2),:);
                v2 = vertexArray(triangArray(j,2),:) - vertexArray(triangArray(j,1),:);


                nvek=cross(v2,v1); %calculate normal vectors
                nvek=nvek/norm(nvek);
                nnew(j,:) = nvek;
                dot(nvek, normalArray(j,:))
                if dot(nvek, normalArray(j,:))<0
                    triangOrder(j,:)=[triangArray(j,3), triangArray(j,2), triangArray(j,1)];
                    nnew(j,:)=-nvek;
                end

            end

        end

        function [contactPtArray, normalArray, stlID, stlMinDist] = findRayIntersect(obj, orig, dir, includeOrigin, vargin)
            %%FINDRAYINTERSECT Return the first point of intersection
            %%created along a vector from a point in space. Include which
            %%STL the ray intersects and the contact point and surface
            %%normal at intersection

            %If the size of the start point array is incompatible with the
            %number of vectors then raise a warning

            contactPtArray = nan(1,3);
            normalArray = nan(1,3);
            stlID = -1;
            stlMinDist = inf;


            nRay = size(orig,1);
            %Make a set of all faces to check
            nSTL = length(obj.FBT);
            for j = 1:nRay
                for i = 1:nSTL
                    faceArray = obj.FBT{i}.ConnectivityList;
                    vertArray = obj.FBT{i}.Points;
                    vert0 = vertArray(faceArray(:,1),:);
                    vert1 = vertArray(faceArray(:,2),:);
                    vert2 = vertArray(faceArray(:,3),:);
                    [intersect, dist, ~, ~, xcoor] = TriangleRayIntersection(orig(j,:), dir(j,:), vert0, vert1, vert2, vargin{:});
                    intersectID = find(intersect);
                    


                    if intersectID
                        %any(contains(vargin, 'line'))
                        %Find the closest positive intersection
                        intersectDist = dist(intersectID);
                        
                        %Exclude intersections that are at the origin
                        if ~includeOrigin
                            intersectID(abs(intersectDist)<1e-5) = [];
                        end

                        if isempty(intersectID)
                            return
                        else
                            intersectDist = dist(intersectID);
                        end


                        [~, closeID] = min(abs(intersectDist));
%                         closeDist = intersectDist(closeID);
%                         stlID = i;
%                         faceID = intersectID(closeID);
%                         contactPtArray = xcoor(faceID, :);
%                         normalArray = faceNormal(obj.FBT{i}, faceID);
                        closeDist = intersectDist(closeID);
                        if closeDist < stlMinDist
                            stlMinDist = closeDist;
                            stlID = i;
                            faceID = intersectID(closeID);
                            contactPtArray = xcoor(faceID, :);
                            normalArray = faceNormal(obj.FBT{i}, faceID);
                        end
                    end
                end
            end




        end
    end

end

