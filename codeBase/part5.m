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

% Define trajectory segments based on proportional time splits
segment1 = 1:round(0.25 * n); % 0% to 25%
segment2 = round(0.25 * n) + 1:round(0.5 * n); % 25% to 50%
segment3 = round(0.5 * n) + 1:round(0.75 * n); % 50% to 75%
segment4 = round(0.75 * n) + 1:n; % 75% to 100%

% Generate trajectory
trajectory = zeros(length(t), 3);
trajectory(segment1, :) = linear_trajectory(waypoints(1, :), waypoints(2, :), 0, 10, t(segment1));
trajectory(segment2, :) = linear_trajectory(waypoints(2, :), waypoints(3, :), 10, 20, t(segment2));
trajectory(segment3, :) = linear_trajectory(waypoints(3, :), waypoints(4, :), 20, 30, t(segment3));
trajectory(segment4, :) = circular_trajectory(waypoints(4, :), waypoints(5, :), circle_center, radius, 30, 40, t(segment4));

% Initialize joint variables
q = zeros(4, length(t)); % Assume 4 DOF for SCARA
q(:, 1) = [0; 0; 0.5; 0]; % Initial guess for joint variables

% Define a target range for theta4 (q4)
theta4_start = 0;          % Start angle for q4
theta4_end = 2 * pi;       % End angle for q4
theta4_rate = (theta4_end - theta4_start) / total_time; % Increment per second

% Perform Euler integration
for i = 1:length(t)-1
    % Current end-effector position
    p_current = forward_kinematics(q(:, i));
    
    % Desired end-effector position
    p_desired = trajectory(i + 1, :);
    
    % Compute error
    error = p_desired - p_current';
    
    % Compute Jacobian
    J = compute_jacobian(q(:, i));
    
    % Inverse Jacobian method
    q_dot_inv = pinv(J) * error'; % Use only the Cartesian position error 
    
    % Update q4 explicitly with a linear variation
    q_dot_inv(4) = theta4_rate; % Linear variation for q4
    
    % Update joint variables
    q(:, i + 1) = q(:, i) + dt * q_dot_inv;
end

% Plot trajectory and joint variables
figure;
subplot(2, 1, 1);
plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'r');
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
