classdef plotting
    methods(Static)
        function formation(x, y, z, a, b, cz, title_text, figure_handle, cx, cy)
            if nargin < 8
                figure_handle = gcf;
            else
                figure(figure_handle);
            end
            clf(figure_handle);

            theta = linspace(0, 2*pi, 100);
            x_base = cx + a * cos(theta) * 1.01;
            y_base = cy + b * sin(theta) * 1.01;
            plot3(x_base, y_base, cz * ones(size(theta)), 'k--', 'LineWidth', 1.5, 'Color', [0 0 0 0.5]);
        
            hold on;
        
            n_drones = length(x);
            for i = 1:n_drones
                plot3(x(i), y(i), z(i), 'bo', 'MarkerSize', 6, 'LineWidth', 2);
                text(x(i), y(i), (z(i) + 0.5), string(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 8.0, 'Color', [0 0 1 1]);
                if i > 1
                    plot3([x(i), x(i-1)], [y(i), y(i-1)], [z(i), z(i-1)], '-', 'LineWidth', 1.5, 'Color', 'r');
                else
                    plot3([x(i), x(n_drones)], [y(i), y(n_drones)], [z(i), z(n_drones)], '-', 'LineWidth', 1.5, 'Color', 'r');
                end
        
                if i < n_drones
                    plot3([x(i), x(i+1)], [y(i), y(i+1)], [z(i), z(i+1)], '-', 'LineWidth', 1.5, 'Color', 'r');
                else
                    plot3([x(i), x(1)], [y(i), y(1)], [z(i), z(1)], '-', 'LineWidth', 1.5, 'Color', 'r');
                end
            end
        
            grid on;
            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');
            
%             if a > b
%                 set_limit = a + (a * 0.5);
%             else
%                 set_limit = b + (b * 0.5);
%             end
%             
%             if a == b
%                 set_limit = a + (a * 0.5);
%             end
%         
%             axis([-set_limit set_limit -set_limit set_limit 0 10]);
            title(title_text);
            view(3);
        end
    end
end