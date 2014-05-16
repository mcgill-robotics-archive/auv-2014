classdef point
    properties
        x           % x coordinate      m
        y           % y coordinate      m
        z           % z coordinate      m
    end

    methods
        function obj = point(x,y,z)
            % constructor
            if (nargin == 2)
                obj.x = x;
                obj.y = y;
                obj.z = 0;                
            elseif (nargin == 3)
                obj.x = x;
                obj.y = y;
                obj.z = z;
            else
                obj.x = 0;
                obj.y = 0;
                obj.z = 0;
            end
        end
    end
end
