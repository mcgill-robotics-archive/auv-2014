classdef receiver
    properties
        pos         % coordinates       m
        time        % time difference   s
    end

    methods
        function obj = receiver(x,y,z)
            % constructor
            if (nargin > 0)
                obj.pos = point(x,y,z);
            else
                obj.pos = point(0,0,0);
            end
            obj.time = 0;
        end

        function obj = time_of_travel(obj,pos,speed)
            % compute time of travel
            obj.time = norm([pos.x-obj.pos.x pos.y-obj.pos.y pos.z-obj.pos.z]) / speed;
        end
    end
end
