classdef array
    properties
        size        % number of receivers
        receivers   % list of receivers
        speed       % speed of sound    m/s
        solution    % sound source location
        error       % error from solution
    end
    
    methods
        function obj = array(size,speed)
            % constructor
            if (nargin > 0)
                obj.size = size;
                obj.receivers = receiver.empty(size,0);
                obj.receivers(1) = receiver();
                obj.speed = speed;
                obj.solution = point();
                obj.error = 0;
            end
        end

        function obj = time_difference(obj,pos)
            % compute time difference of each receiver
            obj.receivers(1) = obj.receivers(1).time_of_travel(pos,obj.speed);
            for i = 2:obj.size
                obj.receivers(i) = obj.receivers(i).time_of_travel(pos,obj.speed);
                obj.receivers(i).time = obj.receivers(1).time ...
                                      - obj.receivers(i).time;
            end
            obj.receivers(1).time = 0;
        end

        function obj = compute_error(obj,pos)
            % compute time difference of each receiver
            obj.error = norm([obj.solution.x-pos.x obj.solution.y-pos.y obj.solution.z]) ...
                      / norm([pos.x pos.y]);
        end
    end
end
