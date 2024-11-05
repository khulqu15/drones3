function virtual_leaders(block)
% Level-2 MATLAB S-Function untuk menghitung posisi dan kecepatan rata-rata dari sekelompok drone sebagai virtual leader,
% dengan posisi dan kecepatan referensi sebagai target.

setup(block);

function setup(block)
    % Pengaturan dasar S-Function
    block.NumInputPorts = 1;    % Input: matriks posisi drone (100x3)
    block.NumOutputPorts = 2;   % Output: posisi leader (1x3), kecepatan leader (1x3)

    % Set up input port type and fix dimensions
    block.InputPort(1).Dimensions = [100, 3];  % Fixed dimensions 100x3
    block.InputPort(1).DatatypeID = 0;         % Double (0 represents double)
    block.InputPort(1).DirectFeedthrough = true;

    % Set up output port dimensions and type
    block.OutputPort(1).Dimensions = [1, 3];   % Position of virtual leader (1x3)
    block.OutputPort(1).DatatypeID = 0;        % Double
    block.OutputPort(2).Dimensions = [1, 3];   % Velocity of virtual leader (1x3)
    block.OutputPort(2).DatatypeID = 0;        % Double

    % Set up parameters (leader_ref_position and leader_ref_velocity)
    block.NumDialogPrms = 2;   % leader_ref_position (1x3) and leader_ref_velocity (1x3)

    % Register sample times
    block.SampleTimes = [0.1 0];  % Sample time of 0.1 seconds

    % Specify block methods
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);
    block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortSamplingMode);
end

function DoPostPropSetup(block)
    % Initialize internal states for storing previous positions as a vector
    block.NumDworks = 1;
    block.Dwork(1).Name = 'prev_position';
    block.Dwork(1).Dimensions = 3;             % Set to 3 (vector of 3 elements)
    block.Dwork(1).DatatypeID = 0;             % double
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
end

function SetInputPortSamplingMode(block, port, mode)
    % Set the sampling mode for the input port
    block.InputPort(port).SamplingMode = mode;

    % Set the sampling mode for the output ports to match the input mode
    block.OutputPort(1).SamplingMode = mode;
    block.OutputPort(2).SamplingMode = mode;
end

function InitializeConditions(block)
    % Initialize previous position with zeros (initial position as vector)
    block.Dwork(1).Data = [0; 0; 0]; % Column vector of zeros
end

function Outputs(block)
    % Retrieve input data (100x3 position matrix)
    positions = block.InputPort(1).Data;

    % Retrieve reference position and velocity from dialog parameters
    leader_ref_position = block.DialogPrm(1).Data;
    leader_ref_velocity = block.DialogPrm(2).Data;

    % Filter out empty rows (assume rows with all zeros are empty)
    valid_positions = positions(any(positions, 2), :);  % Select non-zero rows

    % Calculate the average position (virtual leader position)
    if isempty(valid_positions)
        virtual_leader_position = zeros(1, 3); % Default position if no valid positions
    else
        virtual_leader_position = mean(valid_positions, 1); % Take mean along rows
    end

    % Calculate the average velocity (rate of change of position)
    prev_position = block.Dwork(1).Data(:).';  % Retrieve previous position as row vector
    delta_t = block.SampleTimes(1);            % Time step
    virtual_leader_velocity = (virtual_leader_position - prev_position) / delta_t;

    % Adjust virtual leader position and velocity toward the reference
    position_gain = 0.1;  % Gain for position adjustment towards reference
    velocity_gain = 0.1;  % Gain for velocity adjustment towards reference

    % Move virtual leader's position and velocity towards reference values
    virtual_leader_position = virtual_leader_position + position_gain * (leader_ref_position - virtual_leader_position);
    virtual_leader_velocity = virtual_leader_velocity + velocity_gain * (leader_ref_velocity - virtual_leader_velocity);

    % Output the virtual leader position and velocity as row vectors
    block.OutputPort(1).Data = virtual_leader_position(:).';   % Position output (1x3)
    block.OutputPort(2).Data = virtual_leader_velocity(:).';   % Velocity output (1x3)
end

function Update(block)
    % Update the previous position with the current position for the next step
    block.Dwork(1).Data = block.OutputPort(1).Data(:);  % Store as column vector
end

end
