function plotting_simul(block)
% Level-2 MATLAB S-Function untuk memplot formasi drone dalam 3D.

setup(block);

function setup(block)
    % Pengaturan dasar S-Function
    block.NumInputPorts = 1;    % Input matriks posisi drone (Nx3)
    block.NumOutputPorts = 0;   % Tidak ada output

    % Atur ukuran input port
    block.SetPreCompInpPortInfoToDynamic;
    block.InputPort(1).DimensionsMode = 'Fixed'; % Ubah ke ukuran tetap
    
    % Menentukan apakah input bersifat langsung
    block.InputPort(1).DirectFeedthrough = true;

    % Mengatur waktu sampel
    block.SampleTimes = [0.1 0]; % Sampel setiap 0.1 detik (disesuaikan)

    % Registrasi metode callback
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
end

function SetInpPortDims(block, port, dims)
    % Pengaturan dimensi input port
    block.InputPort(port).Dimensions = dims;
end

function Outputs(block)
    % Ekstraksi data dari input block
    positions = block.InputPort(1).Data;
    
    % Periksa dimensi input
    num_elements = numel(positions);
    if mod(num_elements, 3) ~= 0
        error('Input must be a 1x(3N) vector, where N is the number of drones.');
    end

    % Hitung jumlah drone
    n_drones = num_elements / 3;

    % Ekstrak koordinat x, y, z
    x = positions(1:n_drones);
    y = positions(n_drones + 1:2*n_drones);
    z = positions(2*n_drones + 1:3*n_drones);

    % Parameter untuk plot lingkaran dasar mengelilingi drone
    cx = mean(x); 
    cy = mean(y); 
    cz = mean(z);
    % Radius lingkaran sedikit lebih besar dari jangkauan drone
    r = max(sqrt((x - cx).^2 + (y - cy).^2)) * 1.1;  % Tambahkan sedikit margin

    % Buat plot 3D
    figure(1); 
    clf;
    hold on;  % Pastikan hold on tepat setelah clf untuk mempertahankan plot berikutnya

    % Gambar lingkaran dasar mengelilingi formasi drone dengan warna abu-abu
    theta = linspace(0, 2 * pi, 100);
    x_base = cx + r * cos(theta);
    y_base = cy + r * sin(theta);
    plot3(x_base, y_base, cz * ones(size(theta)), '--', 'LineWidth', 2, 'Color', [0.5, 0.5, 0.5]); % Lingkaran dasar abu-abu
    
    % Plot posisi drone dan garis antar-drone
    for i = 1:n_drones
        plot3(x(i), y(i), z(i), 'bo', 'MarkerSize', 6, 'LineWidth', 2);
        text(x(i), y(i), z(i) + 0.5, string(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 8.0, 'Color', [0 0 1 1]);
        
        if i < n_drones
            plot3([x(i), x(i+1)], [y(i), y(i+1)], [z(i), z(i+1)], '-', 'LineWidth', 1.5, 'Color', 'r');
        else
            plot3([x(i), x(1)], [y(i), y(1)], [z(i), z(1)], '-', 'LineWidth', 1.5, 'Color', 'r');
        end
    end

    % Set properties for the plot
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Posisi Drone dalam Formasi 3D');
    view(3);
    hold off;

    % Force MATLAB to update the plot
    drawnow;
end
end
