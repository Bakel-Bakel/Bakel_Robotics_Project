clc;
clear;

% Define the points (key waypoints)
P0 = [0.6; 0.4; 0]; % Start point
P1 = [0.6; 0.5; 0]; % First point
P2 = [0.7; 0.4; 0]; % Second point
P3 = [0.85; 0.35; 0]; % Third point
P4 = [0.95; 0.0; 0]; % End point

% Radius of the circular section
radius = norm(P4 - P3) / 2;

% Define the motion parameters for each segment
s1 = Position(0, 10, norm(P1 - P0)); % Segment 1 (linear)
s2 = Position(10, 20, norm(P2 - P1)); % Segment 2 (linear)
s3 = Position(20, 30, norm(P3 - P2)); % Segment 3 (linear)
s4 = Position(30, 40, pi * radius); % Segment 4 (circular)

% Calculate the slope and center for the circular motion
slope = (P4(2) - P3(2)) / (P4(1) - P3(1));
angle1 = atan(slope); % Slope angle
c = [P3(1) + radius * cos(angle1); % Center x-coordinate
     P3(2) + radius * sin(angle1); % Center y-coordinate
     0]; % Center z-coordinate

% Rotation matrix for circular motion
R = [-cos(angle1), sin(angle1), 0;
     -sin(angle1), -cos(angle1), 0;
      0,           0,           1];

% Time parameters
ti = linspace(0, 40, 100); % Full trajectory time
traj1 = zeros(100, 3); % Initialize trajectory
tz = zeros(100, 3); % Time stamps for z-axis (optional for 3D plot)

% Trajectory generation
for i = 1:100
    t = ti(i); % Current time
    if t <= 10
        % Segment 1: Linear from Pi to P1
        q1 = s1(4) * t^3 + s1(3) * t^2 + s1(2) * t + s1(1);
        PF1 = P0 + q1 * (P1 - P0) / norm(P1 - P0);
        traj1(i, :) = PF1';
    elseif t > 10 && t <= 20
        % Segment 2: Linear from P1 to P2
        q2 = s2(4) * t^3 + s2(3) * t^2 + s2(2) * t + s2(1);
        PF2 = P1 + q2 * (P2 - P1) / norm(P2 - P1);
        traj1(i, :) = PF2';
    elseif t > 20 && t <= 30
        % Segment 3: Linear from P2 to P3
        q3 = s3(4) * t^3 + s3(3) * t^2 + s3(2) * t + s3(1);
        PF3 = P2 + q3 * (P3 - P2) / norm(P3 - P2);
        traj1(i, :) = PF3';
    elseif t > 30 && t <= 40
        % Segment 4: Circular motion from P3 to P4
        q4 = s4(4) * t^3 + s4(3) * t^2 + s4(2) * t + s4(1);
        PF4 = c + R * [radius * cos(q4 / radius);
                       radius * sin(q4 / radius);
                       0];
        traj1(i, :) = PF4';
    end
    tz(i, :) = i; % Optional time stamps for 3D plot
end

% Workspace Parameters
theta1_range = linspace(deg2rad(-90), deg2rad(90), 100); % Joint 1 range
theta2_range = linspace(deg2rad(-90), deg2rad(45), 100); % Joint 2 range
a1 = 0.5; % Length of link 1
a2 = 0.5; % Length of link 2

[X, Y] = meshgrid(theta1_range, theta2_range);
X_pos = a1 * cos(X) + a2 * cos(X + Y);
Y_pos = a1 * sin(X) + a2 * sin(X + Y);

% Plot workspace and trajectory
figure;
hold on;

% Plot workspace
plot(X_pos, Y_pos, 'b.', 'MarkerSize', 2);

% Plot planned trajectory
plot(traj1(:, 1), traj1(:, 2), 'r-', 'LineWidth', 2);

% Mark waypoints
scatter([P0(1), P1(1), P2(1), P3(1), P4(1)], ...
        [P0(2), P1(2), P2(2), P3(2), P4(2)], 'go', 'filled');

% Annotate waypoints
text(P0(1), P0(2), 'P_0', 'FontSize', 10, 'Color', 'k');
text(P1(1), P1(2), 'P_1', 'FontSize', 10, 'Color', 'k');
text(P2(1), P2(2), 'P_2', 'FontSize', 10, 'Color', 'k');
text(P3(1), P3(2), 'P_3', 'FontSize', 10, 'Color', 'k');
text(P4(1), P4(2), 'P_4', 'FontSize', 10, 'Color', 'k');

% Labels and visualization settings
xlabel('X-axis (meters)');
ylabel('Y-axis (meters)');
title('SCARA Robot Workspace and Trajectory');
axis equal;
grid on;

hold off;

% Local Function: Position (Cubic Polynomial Coefficients)
function Position = Position(ti, tf, df)
    % Inputs: ti (start time), tf (end time), df (displacement)
    % Outputs: Position (polynomial coefficients for cubic trajectory)
    input1 = [0; 0; df; 0]; % Boundary conditions: [start pos, start vel, end pos, end vel]
    matris1 = [1, ti, ti^2, ti^3; % Time polynomial matrix
               0, 1, 2 * ti, 3 * ti^2;
               1, tf, tf^2, tf^3;
               0, 1, 2 * tf, 3 * tf^2];
    Position = linsolve(matris1, input1); % Solve for coefficients
end
