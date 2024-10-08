classdef flocking
    properties
        separation_weight  % Weight for separation force
        alignment_weight   % Weight for alignment force
        cohesion_weight    % Weight for cohesion force
        neighbor_radius    % Radius to consider other drones as neighbors
        max_velocity       % Maximum allowed velocity for each drone
    end
    
    methods
        function obj = flocking(separation_weight, alignment_weight, cohesion_weight, neighbor_radius, max_velocity)
            % Constructor to initialize flocking parameters
            obj.separation_weight = separation_weight;
            obj.alignment_weight = alignment_weight;
            obj.cohesion_weight = cohesion_weight;
            obj.neighbor_radius = neighbor_radius;
            obj.max_velocity = max_velocity;
        end
        
        function [separation_force, alignment_force, cohesion_force] = calculate_forces(obj, drone_positions, drone_velocities, drone_index)
            % Calculate flocking forces for the drone at `drone_index`
            % Inputs:
            %   drone_positions: N x 3 matrix of drone positions
            %   drone_velocities: N x 3 matrix of drone velocities
            %   drone_index: index of the current drone being processed
            
            % Get the current drone's position and velocity
            current_position = drone_positions(drone_index, :);
            current_velocity = drone_velocities(drone_index, :);
            
            % Initialize forces
            separation_force = [0, 0, 0];
            alignment_force = [0, 0, 0];
            cohesion_force = [0, 0, 0];
            
            num_neighbors = 0;
            
            % Loop through all drones to find neighbors
            for i = 1:size(drone_positions, 1)
                if i ~= drone_index
                    distance = norm(drone_positions(i, :) - current_position);
                    if distance < obj.neighbor_radius
                        % Separation: steer away from neighbors that are too close
                        separation_force = separation_force + (current_position - drone_positions(i, :)) / (distance^2);
                        
                        % Alignment: align velocity with neighbors
                        if i <= size(drone_velocities, 1)
                            alignment_force = alignment_force + drone_velocities(i, :);
                        end
                        
                        % Cohesion: steer towards the average position of neighbors
                        cohesion_force = cohesion_force + drone_positions(i, :);
                        
                        num_neighbors = num_neighbors + 1;
                    end
                end
            end
            
            % Normalize and scale the forces
            if num_neighbors > 0
                % Calculate average alignment and cohesion
                alignment_force = alignment_force / num_neighbors;
                cohesion_force = cohesion_force / num_neighbors - current_position;
                
                % Apply weights to the forces
                separation_force = obj.separation_weight * separation_force;
                alignment_force = obj.alignment_weight * (alignment_force - current_velocity);
                cohesion_force = obj.cohesion_weight * cohesion_force;
            end
        end
        
        function new_velocity = apply_flocking(obj, current_velocity, separation_force, alignment_force, cohesion_force)
            % Combine the flocking forces to update the drone's velocity
            new_velocity = current_velocity + separation_force + alignment_force + cohesion_force;
            
            % Limit the new velocity to the maximum allowed velocity
            speed = norm(new_velocity);
            if speed > obj.max_velocity
                new_velocity = (new_velocity / speed) * obj.max_velocity;
            end
        end
    end
end
