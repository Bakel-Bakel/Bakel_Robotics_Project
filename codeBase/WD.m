% Workspace Parameters
theta1_range = linspace(deg2rad(-90), deg2rad(90), 50); % Joint 1 range
theta2_range = linspace(deg2rad(-90), deg2rad(45), 50); % Joint 2 range
d3_range = linspace(0.25, 1, 20); % Prismatic joint range (Z-axis)
theta4_range = linspace(deg2rad(-360), deg2rad(360), 20); % End-effector orientation range
a1 = 0.5; % Length of link 1
a2 = 0.5; % Length of link 2

% Initialize arrays for storing workspace points
X_pos = [];
Y_pos = [];
Z_pos = [];

% Compute dexterous workspace
for theta1 = theta1_range
    for theta2 = theta2_range
        for d3 = d3_range
            for theta4 = theta4_range
                % Compute the position of the end-effector origin
                x = a1 * cos(theta1) + a2 * cos(theta1 + theta2);
                y = a1 * sin(theta1) + a2 * sin(theta1 + theta2);
                z = d3; % Prismatic joint movement
                % Store position for all orientations
                X_pos = [X_pos; x];
                Y_pos = [Y_pos; y];
                Z_pos = [Z_pos; z];
            end
        end
    end
end

% Plot the dexterous workspace in 3D
figure;
scatter3(X_pos, Y_pos, Z_pos, 1, 'b', 'filled'); % 3D scatter plot
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('3D Dexterous Workspace of SCARA Robot');
grid on;
axis equal;
