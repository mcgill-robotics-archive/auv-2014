classdef point
    properties
        x           % x coordinate      m
        y           % y coordinate      m
    end

    methods
        function obj = point(x,y)
        % constructor
            if (nargin > 0)
                obj.x = x;
                obj.y = y;
            end
        end
    end
end
