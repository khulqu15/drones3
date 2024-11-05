function flocking_alpha_lattice(block)
% S-Function Level-2 MATLAB untuk kontrol flocking menggunakan metode alpha-lattice dengan ukuran tetap.

setup(block);

function setup(block)
    % Pengaturan dasar S-Function
    block.NumInputPorts = 1;   % Input: Matriks posisi drone (Nx3)
    block.NumOutputPorts = 1;  % Output: Gaya yang diterapkan pada setiap drone (Nx3)

    % Set input port properties - fixed-size
    block.InputPort(1).Dimensions = [100, 3];  % Misalnya, tetapkan ukuran tetap 100x3 (pastikan cocok dengan bearing_measurement)
    block.InputPort(1).DatatypeID = 0;         % Double
    block.InputPort(1).DirectFeedthrough = true;

    % Set output port properties - fixed-size
    block.OutputPort(1).Dimensions = [100, 3]; % Output port mengikuti ukuran tetap input
    block.OutputPort(1).DatatypeID = 0;        % Double

    % Set number of parameters for control gains
    block.NumDialogPrms = 4;  % separation_gain, alignment_gain, cohesion_gain, neighbor_radius

    % Register sample times
    block.SampleTimes = [0.1 0];  % Sample every 0.1 seconds

    % Specify block methods
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs', @Outputs);
end

function DoPostPropSetup(block)
    % Initial setup after propagation
    block.NumDworks = 1;
    block.Dwork(1).Name = 'forces';
    block.Dwork(1).Dimensions = 3; % 3 dimensions for forces
    block.Dwork(1).DatatypeID = 0; % double
    block.Dwork(1).Complexity = 'Real';
end

function InitializeConditions(block)
    % Initialize forces to zero
    block.Dwork(1).Data = zeros(1, 3);
end

function Outputs(block)
    % Retrieve input positions (100x3 fixed matrix)
    positions = block.InputPort(1).Data;
    n_drones = size(positions, 1);  % Number of drones

    % Retrieve control gains and parameters from dialog parameters
    separation_gain = block.DialogPrm(1).Data;
    alignment_gain = block.DialogPrm(2).Data;
    cohesion_gain = block.DialogPrm(3).Data;
    neighbor_radius = block.DialogPrm(4).Data;

    % Initialize force array for each drone
    forces = zeros(n_drones, 3);

    % Calculate forces for each drone
    for i = 1:n_drones
        pos_i = positions(i, :);
        
        separation_force = [0, 0, 0];
        alignment_force = [0, 0, 0];
        cohesion_force = [0, 0, 0];
        num_neighbors = 0;

        % Iterate over all drones to calculate forces
        for j = 1:n_drones
            if i ~= j
                pos_j = positions(j, :);
                distance = norm(pos_j - pos_i);

                if distance < neighbor_radius
                    % Separation force: repel from neighbors within radius
                    separation_force = separation_force + (pos_i - pos_j) / distance^2;

                    % Alignment force: align velocity with neighbors
                    alignment_force = alignment_force + pos_j; % assuming positions as velocity proxy

                    % Cohesion force: steer towards average position of neighbors
                    cohesion_force = cohesion_force + pos_j;

                    num_neighbors = num_neighbors + 1;
                end
            end
        end

        % Compute the average alignment and cohesion if there are neighbors
        if num_neighbors > 0
            alignment_force = alignment_force / num_neighbors - pos_i; % Adjust alignment force
            cohesion_force = cohesion_force / num_neighbors - pos_i;   % Adjust cohesion force

            % Apply the gains to each force
            separation_force = separation_gain * separation_force;
            alignment_force = alignment_gain * alignment_force;
            cohesion_force = cohesion_gain * cohesion_force;
        end

        % Total force for each drone
        forces(i, :) = separation_force + alignment_force + cohesion_force;
    end

    % Set the output force matrix
    block.OutputPort(1).Data = forces;
end

end
