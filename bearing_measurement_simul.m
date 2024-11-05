function bearing_measurement(block)
    setup(block);
end

function setup(block)
    % Register the number of input and output ports
    block.NumInputPorts  = 5;  % r, n_drones, cx, cy, cz
    block.NumOutputPorts = 1;  % posisi (x, y, z) matrix

    % Set up input port dimensions and types
    for i = 1:5
        block.InputPort(i).Dimensions = 1;  % All inputs are scalar values
        block.InputPort(i).DatatypeID = 0;  % Double (0 represents double)
        block.InputPort(i).DirectFeedthrough = true;
    end
    
    % Set up default output port dimensions (large enough for maximum possible drones)
    max_drones = 100;  % Adjust this value based on the expected maximum number of drones
    block.OutputPort(1).Dimensions = [max_drones, 3];  % (x, y, z) position matrix

    % Set up parameters (no tunable parameters here)
    block.NumDialogPrms = 0;

    % Register sample times
    block.SampleTimes = [0 0];  % Inherited sample time

    % Specify block methods
    block.RegBlockMethod('Outputs', @Output);
    block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortSamplingMode);
end

function SetInputPortSamplingMode(block, port, mode)
    % Set the sampling mode for each input port
    block.InputPort(port).SamplingMode = mode;
    
    % Set the sampling mode for output ports to match input ports
    block.OutputPort(1).SamplingMode = mode;
end

function Output(block)
    % Retrieve input values
    r = block.InputPort(1).Data;  % Radius of the formation
    n_drones = block.InputPort(2).Data;  % Number of drones
    cx = block.InputPort(3).Data;  % Center x-coordinate
    cy = block.InputPort(4).Data;  % Center y-coordinate
    cz = block.InputPort(5).Data;  % Center z-coordinate

    % Ensure n_drones is an integer and positive
    if ~isnumeric(n_drones) || n_drones <= 0 || mod(n_drones,1) ~= 0
        error('The input n_drones must be a positive integer.');
    end

    % Preallocate the position matrix with max_drones length
    max_drones = block.OutputPort(1).Dimensions(1); % Use the predefined max drones
    position_matrix = NaN(max_drones, 3);  % Initialize with NaN for unused entries

    % Calculate the x, y, z positions only for the actual number of drones
    for i = 1:n_drones
        theta = (2 * pi * (i - 1)) / n_drones;
        x_i = cx + r * cos(theta);
        y_i = cy + r * sin(theta);
        z_i = cz;

        % Store positions in the matrix
        position_matrix(i, :) = [x_i, y_i, z_i];
    end

    % Assign calculated values to output port (without changing dimensions)
    block.OutputPort(1).Data = position_matrix;
end
