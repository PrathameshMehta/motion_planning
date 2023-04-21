classdef OccupancyMapCreator < nav.StateValidator
    %EXAMPLEHELPERFURNITUREINEngineVALIDATOR State validator for furnitures
    %   in the Engine
    
    properties
        %Engine
        Engine
        
        %FurnitureID
        FurnitureID
        
        %ValidationDistance
        ValidationDistance = inf
    end
    
    methods
        function obj = OccupancyMapCreator(stateSpace, initialFurniturePose)
            %EXAMPLEHELPERFURNITUREINEngineVALIDATOR Constructor
            obj@nav.StateValidator(stateSpace);
            
            % Create a Engine
            obj.Engine = MakeEngine(6, 2);
            
            % Add furniture to the Engine
            fn = MakeShaft();
            obj.FurnitureID = obj.Engine.addShaft(fn, initialFurniturePose);
        end
        
        function isValid = isStateValid(obj, state)
            %isStateValid
            isValid = zeros(size(state,1), 1);
            for k = 1:size(state, 1)
                T1 = trvec2tform([state(k,1:3)]);
                T2 = quat2tform([state(k, 4:7)]);
                obj.Engine.shaftsInEngine{obj.FurnitureID}.moveTo(T1*T2);
                inCollision = obj.Engine.checkCollision(obj.FurnitureID);
                if inCollision
                    isValid(k) = false;
                else
                    isValid(k) = true;
                end
            end
        end
        
        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
            %isMotionValid
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            interpStates = obj.StateSpace.interpolate(state1, state2, 0:interval:1);
            
            interpValid = obj.isStateValid(interpStates);
            
            lastValidIdx = find(~interpValid, 1);
            if isempty(lastValidIdx)
                isValid = true;
                lastValid = state2;
            else
                isValid = false;
                lastValid = interpStates(lastValidIdx,:);
            end            
        end

        function newObj = copy(obj)
            %copy
        end
    end
end

