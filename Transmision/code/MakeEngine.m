classdef MakeEngine < handle & ...
                             robotics.core.internal.InternalAccess
    %EXAMPLEHELPERROOM
    
    properties
        %Length Length of the room
        Length
        
        %Width Width of the room
        Width
        
        %Fixtures Collection of room fixtures (like poles, walls)
        Fixtures
        
        %FunituresInRoom Collection of furnitures in the room
        shaftsInEngine
        
        FixedTforms

    end
    
    methods
        function obj = MakeEngine(length, width)
            %EXAMPLEHELPERROOM Constructor
            obj.Length = length;
            obj.Width = width;
            
            % Add poles
           
            obj.Fixtures{1} = collisionCylinder(0.72/2, 6.6);
            obj.Fixtures{2} = collisionCylinder(2/2, 0.5);
            obj.Fixtures{3} = collisionCylinder(1.6/2, 0.5);
            obj.Fixtures{4} = collisionCylinder(2.47/2, 0.5);
            obj.Fixtures{5} = collisionCylinder(2.79/2, 0.5);
            obj.Fixtures{6} = collisionCylinder(2.8/2, 0.5);
        
            angles = [0 pi/2 0];
            obj.FixedTforms{1} = trvec2tform([0, 2.5, 0]) * eul2tform(angles);
            obj.FixedTforms{2} = trvec2tform([2.1, 2.5, 0]) * eul2tform(angles);
            obj.FixedTforms{3} = trvec2tform([1, 2.5, 0]) * eul2tform(angles);
            obj.FixedTforms{4} = trvec2tform([0.3, 2.5, 0]) * eul2tform(angles);
            obj.FixedTforms{5} = trvec2tform([-0.2, 2.5, 0]) * eul2tform(angles);
            obj.FixedTforms{6} = trvec2tform([-2, 2.5, 0]) * eul2tform(angles);
            
            
            obj.Fixtures{1}.Pose = obj.FixedTforms{1};
            obj.Fixtures{2}.Pose = obj.FixedTforms{2};
            obj.Fixtures{3}.Pose = obj.FixedTforms{3};
            obj.Fixtures{4}.Pose = obj.FixedTforms{4};
            obj.Fixtures{5}.Pose = obj.FixedTforms{5};
            obj.Fixtures{6}.Pose = obj.FixedTforms{6};
            
            
            % Add walls
            obj.Fixtures{7} = collisionBox(6.6, 0.1, 4);
            obj.Fixtures{7}.Pose = trvec2tform([0,0, 0]);
            
            obj.Fixtures{8} = collisionBox(0.1, 4, 4);
            obj.Fixtures{8}.Pose = trvec2tform([-6.6/2, 2, 0]); % wall on the left lower half
            
            obj.Fixtures{12} = collisionBox(0.1, 1.6, 1.2);
            obj.Fixtures{12}.Pose = trvec2tform([-6.6/2,4.8, 1.4]); 

            obj.Fixtures{13} = collisionBox(0.1, 1.6, 1.2);
            obj.Fixtures{13}.Pose = trvec2tform([-6.6/2,4.8, -1.4]);

            obj.Fixtures{11} = collisionBox(0.1, 1, 4);
            obj.Fixtures{11}.Pose = trvec2tform([-6.6/2,6, 0]); % wall on the left upper half
            
            obj.Fixtures{9} = collisionBox(0.1, 4, 4);
            obj.Fixtures{9}.Pose = trvec2tform([6.6/2, 2, 0]); % wall on the right lower half

            obj.Fixtures{14} = collisionBox(0.1, 1.6, 1.2);
            obj.Fixtures{14}.Pose = trvec2tform([6.6/2,4.8, 1.4]); 

            obj.Fixtures{15} = collisionBox(0.1, 1.6, 1.2);
            obj.Fixtures{15}.Pose = trvec2tform([6.6/2,4.8, -1.4]);

            obj.Fixtures{10} = collisionBox(0.1, 1, 4);
            obj.Fixtures{10}.Pose = trvec2tform([6.6/2,6, 0]); % wall on the right upper half
        end
        function fnID = addShaft(obj, furniture, initialPose)
            %addFurniture
            fn = furniture;
            fn.moveTo(initialPose);
            obj.shaftsInEngine{end+1} = fn;
            fnID = length(obj.shaftsInEngine);
        end
        
        function show(obj, ax, poses)
            %show
            hold on;
            
            view([-50 40]);
            for k = 1 : length(obj.Fixtures)
                [~, p] = obj.Fixtures{k}.show('Parent', ax);
                p.FaceColor = [0.3 0.3 0.3];
            end
            for k = 1 : length(obj.shaftsInEngine)
                if nargin > 2
                    obj.shaftsInEngine{k}.moveTo(poses{k});
                end
         
                obj.shaftsInEngine{k}.show(ax);
            end
            hold on;

        end
        
        function animateFurnitureMotion(obj, furnitureID, states, ax)
            %animateFurnitureMotion
            hold on;
            view([-50 40]);
            for k = 1 : length(obj.Fixtures)
                [~, p] = obj.Fixtures{k}.show('Parent', ax);
                p.FaceColor = [0.3 0.3 0.3];
            end
            
            set(gcf, 'color', [1 1 1]);
            box(ax, 'on');

            h = [];
            T0 = eye(4);
            axis equal
            for j = 1:size(states,1)
                s = states(j,:);
                T = trvec2tform([s(1) s(2) s(3)]) * quat2tform([s(4) s(5) s(6) s(7)]);
                obj.shaftsInEngine{furnitureID}.moveTo(T);
                if isempty(h)
                    h = obj.shaftsInEngine{furnitureID}.show(ax);
                    T0 = T;
                else
                    h.Matrix = T*robotics.core.internal.SEHelpers.tforminvSE3(T0);
                end
                 
                pause(0.1)
                 
            end
            
            hold on
            plot3(ax, states(:,1), states(:,2),states(:,3), 'r.-', 'linewidth', 3)

        end
        
        function inCollision = checkCollision(obj, furnitureID)
            %checkCollision Check collision for the specified furniture
            inCollision = false;
            numParts = length(obj.shaftsInEngine{furnitureID}.Parts);
            for i = 1:length(obj.Fixtures)
                for j = 1:numParts
                    %cc = checkCollision(obj.Fixtures{i}, obj.shaftsInEngine{furnitureID}.Parts{j});
                    pos1 = obj.Fixtures{i}.Position;
                    quat1 = obj.Fixtures{i}.Quaternion;
                    pos2 = obj.shaftsInEngine{furnitureID}.Parts{j}.Position;
                    quat2 = obj.shaftsInEngine{furnitureID}.Parts{j}.Quaternion;
                    cc = robotics.core.internal.intersect(obj.Fixtures{i}.GeometryInternal, pos1, quat1,...
                                                     obj.shaftsInEngine{furnitureID}.Parts{j}.GeometryInternal, pos2, quat2, 0);
                    if cc == 1
                        inCollision = true;
                        return
                    end
                end
            end
            
        end
        
    end
end