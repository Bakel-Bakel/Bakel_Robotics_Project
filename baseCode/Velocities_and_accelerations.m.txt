clc;
clear;

%% Define the Points (Key Waypoints)
P0 = [0.6; 0.4; 0]; % Start point
P1 = [0.6; 0.5; 0]; % First point
P2 = [0.7; 0.4; 0]; % Second point
P3 = [0.85; 0.35; 0]; % Third point
P4 = [0.95; 0.0; 0]; % End point

% Radius of the circular section
radius = norm(P4 - P3) / 2;

% Time parameters for trajectory planning
t_total = 10; % Total time (seconds)
t_accel = 2; % Acceleration phase duration (seconds)

% Maximum velocity for trapezoidal profile
v_max = 2 * norm(P1 - P0) / t_total; % Example calculation

%% Plan Trajectory Using Trapezoidal Velocity Profile
% Time vector
t = linspace(0, t_total, 500); % Discretized time

% Initialize arrays for position, velocity, and acceleration
q = zeros(size(t)); % Position
v = zeros(size(t)); % Velocity
a = zeros(size(t)); % Acceleration

% Compute the trapezoidal velocity profile
for i = 1:length(t)
    if t(i) <= t_accel
        % Acceleration phase
        a(i) = v_max / t_accel;                % Constant acceleration
        v(i) = a(i) * t(i);                    % Linear increase in velocity
        q(i) = 0.5 * a(i) * t(i)^2;            % Quadratic increase in position
    elseif t(i) <= t_total - t_accel
        % Constant velocity phase
        a(i) = 0;                              % No acceleration
        v(i) = v_max;                          % Constant velocity
        q(i) = q(i-1) + v(i) * (t(i) - t(i-1)); % Linear increase in position
    else
        % Deceleration phase
        a(i) = -v_max / t_accel;               % Constant deceleration
        v(i) = v(i-1) + a(i) * (t(i) - t(i-1)); % Linear decrease in velocity
        q(i) = q(i-1) + v(i) * (t(i) - t(i-1)); % Position approaches end
    end
end

%% Plot Position, Velocity, and Acceleration Profiles

figure;

% Plot position
subplot(3, 1, 1);
plot(t, q, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Trapezoidal Velocity Profile - Position');
grid on;

% Plot velocity
subplot(3, 1, 2);
plot(t, v, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Trapezoidal Velocity Profile - Velocity');
grid on;

% Plot acceleration
subplot(3, 1, 3);
plot(t, a, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Trapezoidal Velocity Profile - Acceleration');
grid on;

%% Generate Trajectory in Workspace and Plot

% Initialize trajectory
traj = zeros(length(t), 3);

for i = 1:length(t)
    if q(i) <= norm(P1 - P0)
        % Segment 1: Linear from P0 to P1
        traj(i, :) = P0' + q(i) * (P1 - P0)' / norm(P1 - P0);
    elseif q(i) <= norm(P1 - P0) + norm(P2 - P1)
        % Segment 2: Linear from P1 to P2
        dq = q(i) - norm(P1 - P0);
        traj(i, :) = P1' + dq * (P2 - P1)' / norm(P2 - P1);
    elseif q(i) <= norm(P1 - P0) + norm(P2 - P1) + norm(P3 - P2)
        % Segment 3: Linear from P2 to P3
        dq = q(i) - norm(P1 - P0) - norm(P2 - P1);
        traj(i, :) = P2' + dq * (P3 - P2)' / norm(P3 - P2);
    else
        % Segment 4: Circular motion from P3 to P4
        dq = q(i) - norm(P1 - P0) - norm(P2 - P1) - norm(P3 - P2);
        theta = dq / radius;
        traj(i, :) = (P3' + radius * [cos(theta), sin(theta), 0]);
    end
end

