classdef receiver
    properties
        pos         % coordinates       m
        time        % time difference   s
    end

    methods
        function obj = receiver(x,y)
        % constructor
            if (nargin > 0)
                obj.pos = point(x,y);
                obj.time = 0;
            end
        end

        function obj = time_of_travel(obj,x,y,speed)
        % compute time of travel
            obj.time = norm([x-obj.pos.x y-obj.pos.y]) / speed;
        end
    end
end
