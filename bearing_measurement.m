classdef bearing_measurement
    methods(Static)
        function [azimuth, elevation] = bearing_circle(n_drones, r, cx, cy, cz)
            azimuth = zeros(1, n_drones);
            elevation = zeros(1, n_drones);

            for i = 1:n_drones
                theta = (2 * pi * (i - 1)) / n_drones;
                x_i = cx + r * cos(theta);
                y_i = cy + r * sin(theta);
                z_i = cz;
                azimuth(i) = atan2d(y_i - cy, x_i - cx);
                elevation(i) = atan2d(z_i - cz, sqrt((x_i - cx)^2 + (y_i - cy)^2));
            end
        end
        
        function [azimuth, elevation] = bearing_ellipse(n_drones, a, b, cx, cy, cz)
            azimuth = zeros(1, n_drones);
            elevation = zeros(1, n_drones);

            for i = 1:n_drones
                theta = (2 * pi * (i - 1)) / n_drones;
                x_i = cx + a * cos(theta);
                y_i = cy + b * sin(theta);
                z_i = cz;

                azimuth(i) = atan2d(y_i - cy, x_i - cx);
                elevation(i) = atan2d(z_i - cz, sqrt((x_i - cx)^2 + (y_i - cy)^2));
            end
        end
    end
end