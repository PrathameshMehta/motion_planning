classdef MakeShaft < handle & ...
                                  robotics.core.internal.InternalAccess
    %EXAMPLEHELPERFURNITURE
    
    properties
        %BoxLength
        BoxLength
        
        %BoxWidth
        BoxWidth
    end
    
    properties
        %Parts This furniture is assembled from three rectangle parts
        Parts
        
        %FixedTforms Fixed transforms for the rectangles relative to the
        %   furniture's coordinates
        FixedTforms
    end
    
    methods
        function obj = MakeShaft()
            %EXAMPLEHELPERFURNITURE Constructor
            y_pos = 0;
%             CYL = collisionCylinder(Radius,Length)
            obj.Parts{1} = collisionCylinder(0.72/2,6.6);
            obj.Parts{2} = collisionCylinder(1.8/2, 3.84);
            obj.Parts{3} = collisionCylinder(2.39/2, 0.6);
            obj.Parts{4} = collisionCylinder(2.12/2, 0.5);
            obj.Parts{5} = collisionCylinder(2.4/2, 0.6);
            obj.Parts{6} = collisionCylinder(2.3/2, 0.2);
            obj.Parts{7} = collisionCylinder(2.59/2, 0.5);
            
            angles = [0 pi/2 0];
            obj.FixedTforms{1} = trvec2tform([0, y_pos, 0]) * eul2tform(angles);
            obj.FixedTforms{2} = trvec2tform([-0.8, y_pos, 0]) * eul2tform(angles);
            obj.FixedTforms{3} = trvec2tform([-2.1, y_pos, 0]) * eul2tform(angles);
            obj.FixedTforms{4} = trvec2tform([-0.85, y_pos, 0]) * eul2tform(angles);
            obj.FixedTforms{5} = trvec2tform([-0.1, y_pos, 0]) * eul2tform(angles);
            obj.FixedTforms{6} = trvec2tform([0.4, y_pos, 0]) * eul2tform(angles);
            obj.FixedTforms{7} = trvec2tform([0.9, y_pos, 0]) * eul2tform(angles);
            
            obj.Parts{1}.Pose = obj.FixedTforms{1};
            obj.Parts{2}.Pose = obj.FixedTforms{2};
            obj.Parts{3}.Pose = obj.FixedTforms{3};
            obj.Parts{4}.Pose = obj.FixedTforms{4};
            obj.Parts{5}.Pose = obj.FixedTforms{5};
            obj.Parts{6}.Pose = obj.FixedTforms{6};
            obj.Parts{7}.Pose = obj.FixedTforms{7};
            
        end
        
        function moveTo(obj, tform)
            %moveTo
            for i = 1:length(obj.Parts)
                T = tform*obj.FixedTforms{i};
                obj.Parts{i}.PoseInternal = T;
                obj.Parts{i}.Position = T(1:3, 4)';
                obj.Parts{i}.Quaternion = rotmToQuaternion(T(1:3, 1:3));
            end
            
        end
        
        function h = show(obj, ax)
            %show
            h = hgtransform(ax);

            hold(ax, 'on');
            [~, p1] = obj.Parts{2}.show('Parent', ax);
            [~, p2] = obj.Parts{1}.show('Parent', ax);
            [~, p3] = obj.Parts{3}.show('Parent', ax);
            [~, p4] = obj.Parts{4}.show('Parent', ax);
            [~, p5] = obj.Parts{5}.show('Parent', ax);
            [~, p6] = obj.Parts{6}.show('Parent', ax);
            [~, p7] = obj.Parts{7}.show('Parent', ax);
            p1.FaceColor = [113 247 201]/255;
            p2.FaceColor = [113 247 201]/255;
            p3.FaceColor = [113 247 201]/255;
            p4.FaceColor = [113 247 201]/255;
            p5.FaceColor = [113 247 201]/255;
            p6.FaceColor = [113 247 201]/255;
            p7.FaceColor = [113 247 201]/255;
            p1.Parent = h;
            p2.Parent = h;
            p3.Parent = h;
            p4.Parent = h;
            p5.Parent = h;
            p6.Parent = h;
            p7.Parent = h;            
            
            
            hold(ax, 'off')

        end
    end
end

function q = rotmToQuaternion(R)
    %rotmToQuaternion
    w = 0.5*sqrt(1 + trace(R));
    if w > 1e-10
        m = 1/(4*w);

        x = m * (R(3,2) - R(2,3));
        y = m * (R(1,3) - R(3,1));
        z = m * (R(2,1) - R(1,2));
    else
        x = 0.5*sqrt(1 + R(1,1) - R(2,2) - R(3,3));
        if x > 1e-10
            m = 1/(4*x);
            y = m * (R(1,2) + R(2,1));
            z = m * (R(1,3) + R(3,1));
            w = m * (R(3,2) - R(2,3));
        else
            y = 0.5*sqrt(1 - R(1,1) + R(2,2) - R(3,3));
            if y > 1e-10
                m = 1/(4*y);
                x = m * (R(2,1) + R(1,2));
                z = m * (R(3,2) + R(2,3));
                w = m * (R(1,3) - R(3,1));
            else
                z = 0.5*sqrt(1 - R(1,1) - R(2,2) + R(3,3));
                m = 1/(4*z);
                x = m * (R(1,3) + R(3,1));
                y = m * (R(3,2) + R(2,3));
                w = m * (R(2,1) - R(1,2));
            end
        end
    end
    
    q = [w x y z];
end