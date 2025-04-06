% Workspace Parameters
theta1_range = linspace(deg2rad(-90), deg2rad(90), 100); % Joint 1 range
theta2_range = linspace(deg2rad(-90), deg2rad(45), 100); % Joint 2 range
d3_range = linspace(0.25, 1, 20); % Prismatic joint range (Z-axis)
a1 = 0.5; % Length of link 1
a2 = 0.5; % Length of link 2

% Generate Meshgrid for Joint Angles
[X, Y] = meshgrid(theta1_range, theta2_range);

% Initialize arrays for storing workspace points
X_pos = [];
Y_pos = [];
Z_pos = [];

% Loop through d3_range to compute 3D positions
for d3 = d3_range
    % Compute the X and Y positions using forward kinematics
    X_tmp = a1 * cos(X) + a2 * cos(X + Y);
    Y_tmp = a1 * sin(X) + a2 * sin(X + Y);
    Z_tmp = d3 * ones(size(X)); % Z positions based on d3
    
    % Store the positions
    X_pos = [X_pos; X_tmp(:)];
    Y_pos = [Y_pos; Y_tmp(:)];
    Z_pos = [Z_pos; Z_tmp(:)];
end

% Plot workspace in 3D
figure;
scatter3(X_pos, Y_pos, Z_pos, 1, 'g', 'filled'); % 3D scatter plot
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('3D Reachable Workspace of SCARA Robot');
grid on;
axis equal;
