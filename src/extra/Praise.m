%% TRAJECTORY

% Defining the waypoints
waypoints = [
    0.6,  0.5, 0.5; % P0 (start)
    0.7,  0.5, 0.6; % P1
    0.7, 0.4, 0.7; % P2
    0.85, 0.35, 0.8; % P3
    0.85, 0.15, 0.9  % P4 (end)
];

% Circular section radius and center
radius = norm(waypoints(5, 1:2) - waypoints(4, 1:2)) / 2;
circle_center = (waypoints(4, 1:2) + waypoints(5, 1:2)) / 2;

% Time parameters
total_time = 40; % Total trajectory time (s)
dt = 0.4; % Time step
t = linspace(0, total_time, total_time / dt);

% Initialize trajectory
trajectory = zeros(length(t), 3);

% Linear segments
trajectory(1:25, :) = linear_trajectory(waypoints(1, :), waypoints(2, :), 0, 10, t(1:25));
trajectory(26:50, :) = linear_trajectory(waypoints(2, :), waypoints(3, :), 10, 20, t(26:50));
trajectory(51:75, :) = linear_trajectory(waypoints(3, :), waypoints(4, :), 20, 30, t(51:75));

% Circular segment
trajectory(76:end, :) = circular_trajectory(waypoints(4, :), waypoints(5, :), circle_center, radius, 30, 40, t(76:end));

% Plot the workspace and trajectory
plot_workspace_and_trajectory(trajectory, waypoints);

% Linear interpolation
function segment = linear_trajectory(P_start, P_end, t_start, t_end, t)
    n = length(t);
    segment = zeros(n, 3);
    for i = 1:n
        tau = (t(i) - t_start) / (t_end - t_start); % Normalized time
        segment(i, :) = P_start + tau * (P_end - P_start);
    end
end

% Circular interpolation
function segment = circular_trajectory(P_start, P_end, center, radius, t_start, t_end, t)
    theta_start = atan2(P_start(2) - center(2), P_start(1) - center(1));
    theta_end = atan2(P_end(2) - center(2), P_end(1) - center(1));
    n = length(t);
    segment = zeros(n, 3);
    for i = 1:n
        tau = (t(i) - t_start) / (t_end - t_start); % Normalized time
        theta = theta_start + tau * (theta_end - theta_start); % Interpolated angle
        segment(i, 1) = center(1) + radius * cos(theta);
        segment(i, 2) = center(2) + radius * sin(theta);
        segment(i, 3) = P_start(3) + tau * (P_end(3) - P_start(3)); % Interpolated z
    end
end

% Workspace and trajectory visualization
function plot_workspace_and_trajectory(trajectory, waypoints)
    % Workspace calculation (SCARA robot example)
    a1 = 0.5; % Link 1 length
a2 = 0.5; % Link 2 length
theta1_range = deg2rad(-90:5:90); % Theta1 in radians
theta2_range = deg2rad(-90:5:45); % Theta2 in radians
d3_range = 0.25:0.05:1; % d3 range from 0.25 to 1

% Generate all combinations of joint variables using meshgrid
[theta1_grid, theta2_grid, d3_grid] = ndgrid(theta1_range, theta2_range, d3_range);

% Calculate Forward Kinematics for all combinations
x = a1 * cos(theta1_grid) + a2 * cos(theta1_grid + theta2_grid);
y = a1 * sin(theta1_grid) + a2 * sin(theta1_grid + theta2_grid);
z = d3_grid;

% Reshape matrices into vectors for plotting
x = x(:);
y = y(:);
z = z(:);

% Plot workspace points
figure;
hold on; 
plot3(x, y, z, '.b', 'MarkerSize', 1); % Workspace points

    % Plot trajectory
    plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'r-', 'LineWidth', 2);

    % Plot waypoints
    scatter3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'go', 'filled');
    for i = 1:size(waypoints, 1)
        text(waypoints(i, 1), waypoints(i, 2), waypoints(i, 3), sprintf('P_%d', i - 1), 'FontSize', 10, 'Color', 'k');
    end

    % Labels and settings
    xlabel('X-axis (m)');
    ylabel('Y-axis (m)');
    zlabel('Z-axis (m)');
    title('Bakel Bakel SCARA Trajectory Planning');
    axis equal;
    grid on;
    view(3); % Set the view to 3D perspective
    hold off;
end