figure_handle = figure;

r = 10;
cx = 0;
cy = 0;
cz = 5;
n_drones = 25;

velocity = 5;
acceleration = 1;
omega = 2 * pi * velocity / r;
d_min = 0.5;
gravity = 9.81;
mass = 1.0;
k = 0.5;
alpha = k * (gravity / mass);

lambda = 2;
eta = 1.5;
k_tsmc = 1.0;
u_max = 10;
controller = twisted_sliding(lambda, eta, k_tsmc, u_max);

x_circle = cx + r * cos((2 * pi * (0:n_drones-1)) / n_drones);
y_circle = cy + r * sin((2 * pi * (0:n_drones-1)) / n_drones);
z_circle = cz * ones(1, n_drones);

a = 100;
b = 50;

x_ellipse = cx + a * cos((2 * pi * (0:n_drones-1)) / n_drones);
y_ellipse = cy + b * sin((2 * pi * (0:n_drones-1)) / n_drones);
z_ellipse = cz * ones(1, n_drones);

leader_velocity = [1, 1, 0];
leader_position = [0, 0, 5];

leader_ref_velocity = [0, 0, 0];
leader_ref_position = [20, 20, 5];

leader = virtual_leader(leader_position, leader_velocity);
dt = 1;
total_time = 50;
time_steps = total_time / dt;

% Initialize drone positions and velocities
drone_positions = [cx + r * cos((2 * pi * (0:n_drones-1)) / n_drones); cy + r * sin((2 * pi * (0:n_drones-1)) / n_drones); cz * ones(1, n_drones)]';
drone_velocities = zeros(n_drones, 3);  % Initially, drones are stationary

% Flocking Controller Initialization
separation_weight = 1.5;
alignment_weight = 1.0;
cohesion_weight = 1.0;
neighbor_radius = 15;  % Radius to consider nearby drones
max_velocity = 5;      % Maximum velocity for drones

flock_controller = flocking(separation_weight, alignment_weight, cohesion_weight, neighbor_radius, max_velocity);

for t = 1:time_steps
    % TSMC for virtual leader control
    [u_x, s_x, ds_x] = controller.control(leader.position(1), leader_ref_position(1), leader.velocity(1), leader_ref_velocity(1));
    [u_y, s_y, ds_y] = controller.control(leader.position(2), leader_ref_position(2), leader.velocity(2), leader_ref_velocity(2));
    [u_z, s_z, ds_z] = controller.control(leader.position(3), leader_ref_position(3), leader.velocity(3), leader_ref_velocity(3));
    leader.velocity = [u_x, u_y, u_z];
    leader = leader.move('forward', dt);

    leader_pos = leader.get_position();
    cx = leader_pos(1);
    cy = leader_pos(2);
    cz = leader_pos(3);

%     for i = 1:n_drones
%         fprintf('Drone %d Position: [%.2f, %.2f, %.2f]\n', i, drone_positions(i, 1), drone_positions(i, 2), drone_positions(i, 3));
%         fprintf('Drone %d Velocity: [%.2f, %.2f, %.2f]\n', i, drone_velocities(i, 1), drone_velocities(i, 2), drone_velocities(i, 3));
%     end

    % Flocking forces update for each drone
    for i = 1:n_drones
        [separation_force, alignment_force, cohesion_force] = flock_controller.calculate_forces(drone_positions, drone_velocities, i);
        % Update velocities using flocking behavior
        drone_velocities(i, :) = flock_controller.apply_flocking(drone_velocities(i, :), separation_force, alignment_force, cohesion_force);
    end

    % Update the positions of the drones based on their velocities
    % drone_positions = drone_positions + drone_velocities * dt;

    % Update drone formation positions based on the leader position
    [azimuth_circle, elevation_circle] = bearing_measurement.bearing_circle(n_drones, r, cx, cy, cz);
    x_circle = cx + r * cos((2 * pi * (0:n_drones-1)) / n_drones);
    y_circle = cy + r * sin((2 * pi * (0:n_drones-1)) / n_drones);
    z_circle = cz * ones(1, n_drones);

    % Plot the updated drone formation
    plotting.formation(x_circle, y_circle, z_circle, r, r, cz, ['Time: ', num2str(t*dt)], figure_handle, cx, cy);

    % Move leader based on time conditions
    if t <= 10
        leader = leader.move('forward', dt);
        if t == 10
            [x_circle, y_circle, z_circle, n_drones] = add_drone_to_formation(x_circle, y_circle, z_circle, n_drones, cx, cy, cz, r, figure_handle);
            drone_positions = [x_circle', y_circle', z_circle'];  % Update positions array
            drone_velocities = [drone_velocities; zeros(1, 3)];
        end
    elseif t < 20
        leader = leader.move('left', dt);
    elseif t < 30
        leader = leader.move('right', dt);
    elseif t < 40
        leader = leader.move('backward', dt);
    end

    % Adding and removing drones at certain times
    if t == 20
        [x_circle, y_circle, z_circle, n_drones] = add_drone_to_formation(x_circle, y_circle, z_circle, n_drones, cx, cy, cz, r, figure_handle);
        drone_positions = [x_circle', y_circle', z_circle'];  % Update positions array
        drone_velocities = [drone_velocities; zeros(1, 3)];
    end
    if t == 30 && n_drones > 1
        drone_index_to_remove = randi(n_drones);  % Randomly select a drone to remove
        [x_circle, y_circle, z_circle, n_drones] = remove_drone_from_formation(x_circle, y_circle, z_circle, n_drones, drone_index_to_remove, figure_handle, cx, cy, r, cz);
        drone_positions(drone_index_to_remove, :) = [];  % Remove the corresponding position
        drone_velocities(drone_index_to_remove, :) = []; % Remove the corresponding velocity
    end

    pause(0.1);
end


function [x_circle, y_circle, z_circle, n_drones] = add_drone_to_formation(x_circle, y_circle, z_circle, n_drones, cx, cy, cz, r, fig_handle)
    % Tambahkan drone baru
    n_drones = n_drones + 1;
    
    theta_all = (2 * pi * (0:n_drones-1)) / n_drones;
    x_targets = cx + r * cos(theta_all);
    y_targets = cy + r * sin(theta_all);
    z_targets = cz * ones(1, n_drones);

    % Posisi awal drone di luar formasi (acak)
    x_new_drone = cx + (r + 10) * cos(2 * pi * rand());
    y_new_drone = cy + (r + 10) * sin(2 * pi * rand());
    z_new_drone = cz;

    x_circle(end + 1) = x_new_drone;
    y_circle(end + 1) = y_new_drone;
    z_circle(end + 1) = z_new_drone;
    
    % Hitung posisi drone baru dalam formasi
    theta_new = (2 * pi * (n_drones - 1)) / n_drones;
    x_target = cx + r * cos(theta_new);
    y_target = cy + r * sin(theta_new);
    z_target = cz;
    
    % Gabungkan drone secara bertahap ke dalam formasi
    num_steps = 10;
    for step = 1:num_steps
        x_new_drone = x_new_drone + (x_target - x_new_drone) / num_steps;
        y_new_drone = y_new_drone + (y_target - y_new_drone) / num_steps;
        z_new_drone = z_target;
        
        % Update posisi formasi drone
        x_circle(end + 1) = x_new_drone;
        y_circle(end + 1) = y_new_drone;
        z_circle(end + 1) = z_new_drone;
        
        for i = 1:n_drones
            x_circle(i) = x_circle(i) + (x_targets(i) - x_circle(i)) / num_steps;
            y_circle(i) = y_circle(i) + (y_targets(i) - y_circle(i)) / num_steps;
            z_circle(i) = z_targets(i);  % ketinggian tetap sama
        end

        % Real-time plotting update
        plotting.formation(x_circle, y_circle, z_circle, r, r, cz, 'Adding Drone', fig_handle, cx, cy);
        pause(0.05);  % Pause to make the movement smooth
        
        % Hapus drone sementara untuk langkah selanjutnya
        x_circle(end) = [];
        y_circle(end) = [];
        z_circle(end) = [];
        
    end
    
    % Simpan drone yang baru ditambahkan ke dalam formasi
    x_circle(end + 1) = x_target;
    y_circle(end + 1) = y_target;
    z_circle(end + 1) = z_target;
end


function [x_circle, y_circle, z_circle, n_drones] = remove_drone_from_formation(x_circle, y_circle, z_circle, n_drones, index, fig_handle, cx, cy, r, cz)
    % Pilih drone yang akan dikeluarkan berdasarkan indeks
    x_remove = x_circle(index);
    y_remove = y_circle(index);
    z_remove = z_circle(index);
    
    % Tentukan posisi tujuan untuk drone yang keluar (misalnya, bergerak menjauh)
    x_target = x_remove + 20;  % Keluar ke kanan
    y_target = y_remove + 20;  % Keluar ke atas
    z_target = z_remove;  % Tetap di ketinggian yang sama
    
    % Gerakkan drone keluar dari formasi
    num_steps = 10;
    for step = 1:num_steps
        x_remove = x_remove + (x_target - x_remove) / num_steps;
        y_remove = y_remove + (y_target - y_remove) / num_steps;
        z_remove = z_target;
        
        % Update posisi drone yang lain dalam formasi
        x_circle(index) = x_remove;
        y_circle(index) = y_remove;
        z_circle(index) = z_remove;
        
        % Real-time plotting update
        plotting.formation(x_circle, y_circle, z_circle, r, r, cz, 'Removing Drone', fig_handle, cx, cy);
        pause(0.05);  % Pause to make the movement smooth
    end
    
    % Hapus drone dari formasi
    x_circle(index) = [];
    y_circle(index) = [];
    z_circle(index) = [];
    n_drones = n_drones - 1;
    
    % Hitung posisi baru semua drone yang tersisa
    theta_all = (2 * pi * (0:n_drones-1)) / n_drones;
    x_targets = cx + r * cos(theta_all);
    y_targets = cy + r * sin(theta_all);
    z_targets = cz * ones(1, n_drones);
    
    % Geser semua drone ke posisi baru
    for step = 1:num_steps
        % Update posisi semua drone
        for i = 1:n_drones
            x_circle(i) = x_circle(i) + (x_targets(i) - x_circle(i)) / num_steps;
            y_circle(i) = y_circle(i) + (y_targets(i) - y_circle(i)) / num_steps;
            z_circle(i) = z_targets(i);  % ketinggian tetap sama
        end
        
        % Real-time plotting update
        plotting.formation(x_circle, y_circle, z_circle, r, r, cz, 'Rearranging Formation', fig_handle, cx, cy);
        pause(0.05);  % Pause to make the movement smooth
    end
end