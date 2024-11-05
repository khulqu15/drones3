classdef virtual_leader
    properties
        position
        velocity
    end

    methods
        function obj = virtual_leader(initial_position, initial_velocity)
            obj.position = initial_position;
            obj.velocity = initial_velocity;
        end

        function obj = move(obj, direction, dt)
            switch direction
                case 'forward'
                    obj.position(1) = obj.position(1) + obj.velocity(1) * dt;
                case 'backward'
                    obj.position(1) = obj.position(1) - obj.velocity(1) * dt;
                case 'left'
                    obj.position(2) = obj.position(2) - obj.velocity(2) * dt;
                case 'right'
                    obj.position(2) = obj.position(2) + obj.velocity(2) * dt;
                case 'up'
                    obj.position(3) = obj.position(3) + obj.velocity(3) * dt;
                case 'down'
                    obj.position(3) = obj.position(3) - obj.velocity(3) * dt;
            end
        end

        function pos = get_position(obj)
            pos = obj.position;
        end
    end
end