clc;
clear;

% Waypoints
waypoints = [
    0.6,  0.5, 0.5; % P0 (start)
    0.7,  0.5, 0.6; % P1
    0.7,  0.4, 0.7; % P2
    0.85, 0.35, 0.8; % P3
    0.85, 0.15, 0.9  % P4 (end)
];

% Circular section radius and center
radius = norm(waypoints(5, 1:2) - waypoints(4, 1:2)) / 2;
circle_center = (waypoints(4, 1:2) + waypoints(5, 1:2)) / 2;

% Trajectory parameters
total_time = 40; % Total trajectory time (s)
dt = 0.001; % Integration time step (1ms)
t = 0:dt:total_time; % Time array
n = length(t); % Total number of time steps

% Generate trajectory
trajectory = zeros(length(t), 3);
trajectory(1:round(0.25 * n), :) = linear_trajectory(waypoints(1, :), waypoints(2, :), 0, 10, t(1:round(0.25 * n)));
trajectory(round(0.25 * n) + 1:round(0.5 * n), :) = linear_trajectory(waypoints(2, :), waypoints(3, :), 10, 20, t(round(0.25 * n) + 1:round(0.5 * n)));
trajectory(round(0.5 * n) + 1:round(0.75 * n), :) = linear_trajectory(waypoints(3, :), waypoints(4, :), 20, 30, t(round(0.5 * n) + 1:round(0.75 * n)));
trajectory(round(0.75 * n) + 1:end, :) = circular_trajectory(waypoints(4, :), waypoints(5, :), circle_center, radius, 30, 40, t(round(0.75 * n) + 1:end));

% Initialize joint variables
q = zeros(4, length(t)); % Assume 4 DOF for SCARA
q(:, 1) = [0; 0; 0.5; 0]; % Initial guess for joint variables

% Joint limits (min/max for each joint)
q_min = [-pi/2; -pi/2; 0.25; -2*pi];
q_max = [pi/2; pi/4; 1; 2*pi];

% Obstacle parameters
obstacle_position = [0.75, 0.3, 0.75]; % Location of the obstacle
obstacle_radius = 0.5; % Radius of the obstacle

% Perform Euler integration
for i = 1:length(t)-1
    % Current end-effector position
    p_current = forward_kinematics(q(:, i));
    
    % Desired end-effector position
    p_desired = trajectory(i + 1, :);
    
    % Compute error in operational space
    error = p_desired - p_current';
    % Relax the z-component (remove z-constraint)
    error(3) = 0; % Uncomment to relax z-component

    % Compute Jacobian
    J = compute_jacobian(q(:, i));
    
    % Modify Jacobian for relaxed orientation component (relax phi)
    J(:, 4) = 0; % Uncomment to relax orientation (phi)
    
    % Pseudo-inverse of the Jacobian
    q_dot = pinv(J) * error';
    
    % Maximize distance from joint limits
    q_center = 0.5 * (q_min + q_max);
    q_range = q_max - q_min;
    weight = diag(1 ./ (q_range - abs(q(:, i) - q_center)).^2);
    q_dot = q_dot + weight * (q_center - q(:, i)); % Weighted adjustment

    % Update joint variables using Euler integration
    q(:, i + 1) = q(:, i) + dt * q_dot;
end

% Plot trajectory and joint variables
figure;
subplot(2, 1, 1);
plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'r');
hold on;
plot3(obstacle_position(1), obstacle_position(2), obstacle_position(3), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Planned Trajectory in Cartesian Space');

subplot(2, 1, 2);
plot(t, q');
grid on;
xlabel('Time (s)');
ylabel('Joint Variables');
legend('q1', 'q2', 'q3', 'q4');
title('Joint Variables Over Time');

%% Functions

% Forward kinematics for SCARA
function p = forward_kinematics(q)
    a1 = 0.5; a2 = 0.5; % Link lengths
    d0 = 0.5;           % Base offset
    theta1 = q(1); theta2 = q(2); d3 = q(3);
    
    x = a1 * cos(theta1) + a2 * cos(theta1 + theta2);
    y = a1 * sin(theta1) + a2 * sin(theta1 + theta2);
    z = d0 + d3;
    p = [x; y; z];
end

% Jacobian for SCARA
function J = compute_jacobian(q)
    a1 = 0.5; a2 = 0.5; % Link lengths
    theta1 = q(1); theta2 = q(2);
    
    J = [
        -a1 * sin(theta1) - a2 * sin(theta1 + theta2), -a2 * sin(theta1 + theta2), 0, 0;
         a1 * cos(theta1) + a2 * cos(theta1 + theta2),  a2 * cos(theta1 + theta2), 0, 0;
         0, 0, 1, 0
    ];
end

% Linear trajectory
function traj = linear_trajectory(p_start, p_end, t_start, t_end, t)
    alpha = (t - t_start) / (t_end - t_start);
    alpha(alpha < 0) = 0; alpha(alpha > 1) = 1;
    traj = (1 - alpha') .* p_start + alpha' .* p_end;
end

% Circular trajectory
function traj = circular_trajectory(p_start, p_end, center, radius, t_start, t_end, t)
    alpha = (t - t_start) / (t_end - t_start);
    alpha(alpha < 0) = 0; alpha(alpha > 1) = 1;
    theta_start = atan2(p_start(2) - center(2), p_start(1) - center(1));
    theta_end = atan2(p_end(2) - center(2), p_end(1) - center(1));
    theta = theta_start + alpha' * (theta_end - theta_start);
    traj = [center(1) + radius * cos(theta), center(2) + radius * sin(theta), linspace(p_start(3), p_end(3), length(t))'];
end
