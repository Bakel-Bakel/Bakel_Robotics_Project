clc
clear


% Workspace Parameters
theta1_range = linspace(deg2rad(-90), deg2rad(90), 100); % Joint 1 range
theta2_range = linspace(deg2rad(-90), deg2rad(45), 100); % Joint 2 range
a1 = 0.5; % Length of link 1
a2 = 0.5; % Length of link 2

[X, Y] = meshgrid(theta1_range, theta2_range);
X_pos = a1 * cos(X) + a2 * cos(X + Y);
Y_pos = a1 * sin(X) + a2 * sin(X + Y);

% Plot workspace
plot(X_pos, Y_pos, 'r.', 'MarkerSize', 1);

% Labels and visualization settings
xlabel('X-axis (meters)');
ylabel('Y-axis (meters)');
title('SCARA Robot Workspace and Trajectory');
axis equal;
grid on;
