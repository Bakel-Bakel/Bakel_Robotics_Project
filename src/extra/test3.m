% Define DH parameters for each link

L1 = Revolute('d', 0, 'a', 0.5, 'alpha', 0);          % Revolute joint (Link 1)
L2 = Revolute('d', 0, 'a', 0.5, 'alpha', pi);            % Revolute joint (Link 2)
L3 = Prismatic('theta', 0, 'a', 0, 'alpha', 0, 'qlim', [0.3, 1]); % Prismatic joint (Link 3)
L4 = Revolute('d', 0, 'a', 0, 'alpha', 0);            % Revolute joint (Link 4)

%% This section displays and plots the robot

% Combine the links into a serial-link robot
Bakel_Scara = SerialLink([L1 L2 L3 L4], 'name', 'Bakel Scara Robot');

% Predefined base transformation parameters 
% This specifies the first link that has a length of 1m
% And rotates the base frame to match that of the question
base_height = 1;              % Base height along the z-axis
base_rotation_x = 0;            % Rotation about the x-axis (0 radians)
base_rotation_y = 0;            % Rotation about the y-axis (0 radians)
base_rotation_z = pi/2;         % Rotation about the z-axis (0 degrees)

% Set the base transformation
Bakel_Scara.base = transl(0, 0, base_height) * ...       % Translate along z-axis
                  trotx(base_rotation_x) * ...          % Rotate about x-axis
                  troty(base_rotation_y) * ...          % Rotate about y-axis
                  trotz(base_rotation_z);               % Rotate about z-axis

% Display the robot structure
% I prefer to add this because it shows the DH table with joints and it is
% easy for me to debug any errors
Bakel_Scara.display();

%Plot the robot with initial joint variables theta 1 =theta 2 = theta 4=0
%and d3 = 0.4
Bakel_Scara.plot([0, 0, 0.4, 0]);
%zlim([0 inf]); % Set z-axis to start from 0 and extend as needed

%% This section is to calculate the forward kinematics
%I used two methods. The robot.fkine() method and the manual transformation
%method

% Define symbolic variables for joint values and link lengths
syms theta1 theta2 d3 theta4 a1 a2 real
syms pi_sym real  % Define pi as a symbolic variable

% Define the symbolic joint variable vector
q_sym = [theta1, theta2, d3, theta4];

% Calculate forward kinematics symbolically
T_sym = Bakel_Scara.fkine(q_sym);

% Extract the symbolic transformation matrix
T_matrix = T_sym.T;  % Extract the 4x4 symbolic matrix

% Simplify and display the transformation matrix
%disp('Symbolic Transformation Matrix (End-effector relative to the base):');
%pretty(simplify(T_matrix));


% Define DH parameters
a1 = a1; alpha1 = 0; d1 = 0;
a2 = a2; alpha2 = pi; d2 = 0;
a3 = 0;   alpha3 = 0;  d3 = 1 - d3;  % Prismatic joint
a4 = 0;   alpha4 = 0;  d4 = 0;

% Transformation matrices for each link
T1 = trotz(theta1) * transl(a1, 0, d1) * trotx(alpha1);
T2 = trotz(theta2) * transl(a2, 0, d2) * trotx(alpha2);
T3 = trotz(0)      * transl(a3, 0, d3) * trotx(alpha3);  % Prismatic joint
T4 = trotz(theta4) * transl(a4, 0, d4) * trotx(alpha4);

% Combine transformations
T = simplify(T1 * T2 * T3 * T4);

% Display the symbolic transformation matrix
disp('Symbolic Transformation Matrix (End-effector relative to the base):');
pretty(T);


%% 
% Define symbolic variables
syms theta1 theta2 d3 theta4 a1 a2 real
syms px py pz phi real  % Desired end-effector position and orientation

% Define the forward kinematics manually
T1 = trotz(theta1) * transl(a1, 0, 0) * trotx(0);
T2 = trotz(theta2) * transl(a2, 0, 0) * trotx(pi);
T3 = transl(0, 0, 1 - d3);  % Prismatic joint
T4 = trotz(theta4);
T_fk = simplify(T1 * T2 * T3 * T4);

% Desired transformation matrix (end-effector pose)
T_desired = transl(px, py, pz) * trotz(phi);

% Extract FK equations for position
fk_position = T_fk(1:3, 4);  % End-effector position [x, y, z]
desired_position = T_desired(1:3, 4);

% Solve for theta1, theta2, d3 (position)
position_eqns = fk_position == desired_position;

% Solve theta1 symbolically
theta1_sol = atan2(py, px);

% Solve for theta2 using the wrist position equation
r = sqrt(px^2 + py^2); % Distance in the xy-plane
cos_theta2 = (r^2 - a1^2 - a2^2) / (2 * a1 * a2);
theta2_sol = acos(cos_theta2);

% Solve for d3
d3_sol = 1 - pz;

% Solve for theta4 (orientation)
fk_orientation = atan2(T_fk(2,1), T_fk(1,1));  % Orientation from FK
desired_orientation = phi;
theta4_sol = desired_orientation - theta1_sol - theta2_sol;

% Display symbolic solutions
disp('Symbolic Inverse Kinematics Solutions:');
disp('theta1:');
disp(theta1_sol);
disp('theta2:');
disp(theta2_sol);
disp('d3:');
disp(d3_sol);
disp('theta4:');
disp(theta4_sol);

%%
% Define robot parameters
a1 = 0.5;  % Length of the first arm
a2 = 0.5;  % Length of the second arm
theta1_min = -pi;  % Joint 1 minimum angle
theta1_max = pi;   % Joint 1 maximum angle
theta2_min = -pi;  % Joint 2 minimum angle
theta2_max = pi;   % Joint 2 maximum angle

% Define resolution for sweeping joint space
num_points = 200;  % Resolution for joint angle sweep
theta1_range = linspace(theta1_min, theta1_max, num_points);
theta2_range = linspace(theta2_min, theta2_max, num_points);

% Initialize arrays for workspace points
x_workspace = [];
y_workspace = [];
boundary_points = [];  % To store key boundary points with joint values

% Generate workspace by sweeping joint space
for theta1 = theta1_range
    for theta2 = theta2_range
        % Forward kinematics to calculate x, y position
        x = a1 * cos(theta1) + a2 * cos(theta1 + theta2);
        y = a1 * sin(theta1) + a2 * sin(theta1 + theta2);
        x_workspace = [x_workspace; x];
        y_workspace = [y_workspace; y];
        
        % Store key boundary points
        if theta1 == theta1_min || theta1 == theta1_max || theta2 == theta2_min || theta2 == theta2_max
            boundary_points = [boundary_points; theta1, theta2, x, y];
        end
    end
end

% Plot the workspace in the x-y plane
figure;
scatter(x_workspace, y_workspace, 1, 'b');  % Plot workspace points
hold on;
scatter(boundary_points(:, 3), boundary_points(:, 4), 50, 'r', 'filled');  % Highlight boundary points
xlabel('X-axis'); ylabel('Y-axis');
title('Accessible Workspace in the X-Y Plane');
grid on; axis equal;

% Annotate boundary points with joint configurations
for i = 1:size(boundary_points, 1)
    text(boundary_points(i, 3), boundary_points(i, 4), sprintf('(%.2f, %.2f)', boundary_points(i, 1), boundary_points(i, 2)), ...
        'FontSize', 8, 'Color', 'k');
end


%%

% Define symbolic variables
syms theta1 theta2 d3 theta4 a1 a2 real

% Forward kinematics
T1 = trotz(theta1) * transl(a1, 0, 0) * trotx(0);
T2 = trotz(theta2) * transl(a2, 0, 0) * trotx(pi);
T3 = transl(0, 0, 1 - d3);  % Prismatic joint
T4 = trotz(theta4);
T_fk = simplify(T1 * T2 * T3 * T4);

% Extract position of the end-effector
p = T_fk(1:3, 4);  % End-effector position

% Initialize Jacobian matrices
J_v = sym(zeros(3, 4));  % Linear velocity Jacobian
J_omega = sym(zeros(3, 4));  % Angular velocity Jacobian

% Compute the Jacobian
z0 = [0; 0; 1];  % Base z-axis
p0 = [0; 0; 0];  % Base origin

% Joint 1 (Revolute)
z1 = T1(1:3, 3);  % z-axis of joint 1
p1 = T1(1:3, 4);  % Position of joint 1
J_v(:, 1) = cross(z0, p - p0);  % Linear velocity contribution
J_omega(:, 1) = z0;  % Angular velocity contribution

% Joint 2 (Revolute)
z2 = T2(1:3, 3);  % z-axis of joint 2
p2 = T2(1:3, 4);  % Position of joint 2
J_v(:, 2) = cross(z1, p - p1);  % Linear velocity contribution
J_omega(:, 2) = z1;  % Angular velocity contribution

% Joint 3 (Prismatic)
z3 = T3(1:3, 3);  % z-axis of joint 3
J_v(:, 3) = z3;  % Linear velocity contribution
J_omega(:, 3) = [0; 0; 0];  % No angular velocity for prismatic joint

% Joint 4 (Revolute)
z4 = T4(1:3, 3);  % z-axis of joint 4
p4 = T4(1:3, 4);  % Position of joint 4
J_v(:, 4) = cross(z3, p - p2);  % Linear velocity contribution
J_omega(:, 4) = z3;  % Angular velocity contribution

% Combine into full Jacobian
J = simplify([J_v; J_omega]);

% Display the symbolic Jacobian
disp('Symbolic Geometric Jacobian:');
pretty(J);

%%

% Define symbolic variables
syms theta1 theta2 d3 theta4 a1 a2 real
syms phi theta psi real  % Euler angles (roll, pitch, yaw)

% Forward kinematics
T1 = trotz(theta1) * transl(a1, 0, 0) * trotx(0);
T2 = trotz(theta2) * transl(a2, 0, 0) * trotx(pi);
T3 = transl(0, 0, 1 - d3);  % Prismatic joint
T4 = trotz(theta4);
T_fk = simplify(T1 * T2 * T3 * T4);

% Extract position and orientation
p = T_fk(1:3, 4);  % End-effector position
R = T_fk(1:3, 1:3);  % End-effector rotation matrix

% Geometric Jacobian
J_g_v = sym(zeros(3, 4));  % Linear velocity Jacobian
J_g_omega = sym(zeros(3, 4));  % Angular velocity Jacobian

% z-axes of each frame
z0 = [0; 0; 1];  % Base frame z-axis
p0 = [0; 0; 0];  % Base frame origin
z1 = T1(1:3, 3);
p1 = T1(1:3, 4);
z2 = T2(1:3, 3);
p2 = T2(1:3, 4);
z3 = T3(1:3, 3);

% Joint 1 (Revolute)
J_g_v(:, 1) = cross(z0, p - p0);
J_g_omega(:, 1) = z0;

% Joint 2 (Revolute)
J_g_v(:, 2) = cross(z1, p - p1);
J_g_omega(:, 2) = z1;

% Joint 3 (Prismatic)
J_g_v(:, 3) = z2;
J_g_omega(:, 3) = [0; 0; 0];

% Joint 4 (Revolute)
J_g_v(:, 4) = cross(z3, p - p2);
J_g_omega(:, 4) = z3;

% Combine into geometric Jacobian
J_g = simplify([J_g_v; J_g_omega]);

% Transformation from angular velocity to Euler angle derivatives
% Assume ZYX Euler angles (roll = phi, pitch = theta, yaw = psi)
T_euler = [
    1, 0, -sin(theta);
    0, cos(phi), cos(theta)*sin(phi);
    0, -sin(phi), cos(theta)*cos(phi)
];

% Analytical Jacobian
J_a = simplify([J_g_v; inv(T_euler) * J_g_omega]);

% Display the analytical Jacobian
disp('Analytical Jacobian:');
pretty(J_a);

%%
% Define 5 points in the workspace (in Cartesian coordinates)
points = [
    0.6, 0.0, 0.5;  % Start point
    0.6, 0.4, 0.5;  % Straight section waypoint
    0.4, 0.6, 0.6;  % Circular section start
    0.0, 0.6, 0.7;  % Circular section end
   -0.4, 0.4, 0.5   % End point
];

% Generate trajectory for the straight section (linear interpolation)
straight_x = linspace(points(1,1), points(2,1), 50);
straight_y = linspace(points(1,2), points(2,2), 50);
straight_z = linspace(points(1,3), points(2,3), 50);

% Generate trajectory for the circular section
theta = linspace(0, pi, 50);  % Circular interpolation (half-circle)
r = 0.1;  % Radius of the circle
circular_x = points(3,1) + r * cos(theta);  % Circle in x-y plane
circular_y = points(3,2) + r * sin(theta);
circular_z = linspace(points(3,3), points(4,3), 50);  % Linear z-change

% Combine the trajectories
x_traj = [straight_x, circular_x, linspace(points(4,1), points(5,1), 50)];
y_traj = [straight_y, circular_y, linspace(points(4,2), points(5,2), 50)];
z_traj = [straight_z, circular_z, linspace(points(4,3), points(5,3), 50)];

% Plot trajectory and waypoints
figure;
scatter3(points(:,1), points(:,2), points(:,3), 100, 'r', 'filled');  % Highlight waypoints
hold on;
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('Trajectory with Waypoints and Dynamic Trace');
grid on;
axis equal;

% Initialize trace for the end-effector
trace = animatedline('LineWidth', 2, 'Color', 'b');  % End-effector trace

% Video writer (optional, comment out if not needed)
video_filename = 'trajectory_animation.mp4';
video_writer = VideoWriter(video_filename, 'MPEG-4');
video_writer.FrameRate = 30;
open(video_writer);

% Simulate the trajectory execution
for i = 1:length(x_traj)
    % Use inverse kinematics to find joint values for each trajectory point
    T_desired = transl(x_traj(i), y_traj(i), z_traj(i));
    q = Bakel_Scara.ikine(T_desired, 'mask', [1 1 1 0 0 0]);  % Solve for q
    Bakel_Scara.plot(q);  % Visualize the robot

    % Add the current end-effector position to the trace
    addpoints(trace, x_traj(i), y_traj(i), z_traj(i));
    drawnow;

    % Capture frame for the video
    frame = getframe(gcf);
    writeVideo(video_writer, frame);

    pause(0.05);  % Adjust pause time for smoother visualization
end

% Finalize video
close(video_writer);

disp(['Trajectory animation saved as: ', video_filename]);


%%
% Define waypoints
waypoints = [
    0.6, 0.0, 0.5;
    0.6, 0.4, 0.5;
    0.4, 0.6, 0.6;
    0.0, 0.6, 0.7;
   -0.4, 0.4, 0.5
];

% Time for waypoints
t_waypoints = linspace(0, 8, size(waypoints, 1));

% Solve IK for waypoints
num_waypoints = size(waypoints, 1);
q_waypoints = nan(num_waypoints, 4);  % Initialize with NaN
q_initial = [0, 0, 0.5, 0];  % Initial guess
valid_indices = [];

for i = 1:num_waypoints
    T_desired = transl(waypoints(i, :));
    try
        q_solution = Bakel_Scara.ikine(T_desired, q_initial, 'mask', [1 1 1 0 0 0]);
        q_waypoints(i, :) = q_solution;  % Store valid solution
        q_initial = q_solution;  % Update initial guess
        valid_indices = [valid_indices, i];  % Log valid index
    catch
        warning('IK failed for waypoint %d. Skipping...', i);
    end
end

% Filter valid waypoints
q_waypoints = q_waypoints(valid_indices, :);
t_waypoints = t_waypoints(valid_indices);

% Ensure valid dimensions for interpolation
if size(q_waypoints, 1) < 2
    error('Not enough valid waypoints for interpolation. Check IK solutions.');
end

% Interpolate joint values
time_resolution = 0.01;
t_interp = linspace(0, 8, 8 / time_resolution);
q_interp = interp1(t_waypoints, q_waypoints, t_interp, 'spline');

% Plot trajectory
figure;
scatter3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 50, 'filled'); hold on;
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('Workspace and Waypoints');
grid on;

% Execute trajectory
figure;
Bakel_Scara.plot(q_interp(1, :));  % Plot initial configuration
for i = 1:size(q_interp, 1)
    if ~isvalid(findobj('Type', 'figure'))  % Ensure figure exists
        error('Figure closed during execution. Stopping trajectory.');
    end

    Bakel_Scara.plot(q_interp(i, :));  % Update robot configuration
    pause(time_resolution);  % Smooth motion
end


%%
% Define waypoints
waypoints = [
    0.6, 0.0, 0.5;
    0.6, 0.4, 0.5;
    0.4, 0.6, 0.6;
    0.0, 0.6, 0.7;
   -0.4, 0.4, 0.5
];

% Time for waypoints
t_waypoints = linspace(0, 8, size(waypoints, 1));

% Solve IK for waypoints
num_waypoints = size(waypoints, 1);
q_waypoints = nan(num_waypoints, 4);  % Initialize with NaN
q_initial = [0, 0, 0.5, 0];  % Initial guess
valid_indices = [];

for i = 1:num_waypoints
    T_desired = transl(waypoints(i, :));
    try
        q_solution = Bakel_Scara.ikine(T_desired, q_initial, 'mask', [1 1 1 0 0 0]);
        q_waypoints(i, :) = q_solution;  % Store valid solution
        q_initial = q_solution;  % Update initial guess
        valid_indices = [valid_indices, i];  % Log valid index
    catch
        warning('IK failed for waypoint %d. Skipping...', i);
    end
end

% Filter valid waypoints
q_waypoints = q_waypoints(valid_indices, :);
t_waypoints = t_waypoints(valid_indices);

% Ensure valid dimensions for interpolation
if size(q_waypoints, 1) < 2
    error('Not enough valid waypoints for interpolation. Check IK solutions.');
end

% Interpolate joint values
time_resolution = 0.01;
t_interp = linspace(0, 8, 8 / time_resolution);
q_interp = interp1(t_waypoints, q_waypoints, t_interp, 'spline');

% Plot trajectory
figure;
scatter3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 50, 'filled'); hold on;
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('Workspace and Waypoints');
grid on;

% Execute trajectory
figure;
Bakel_Scara.plot(q_interp(1, :));  % Plot initial configuration
for i = 1:size(q_interp, 1)
    if ~isvalid(findobj('Type', 'figure'))  % Ensure figure exists
        error('Figure closed during execution. Stopping trajectory.');
    end

    Bakel_Scara.plot(q_interp(i, :));  % Update robot configuration
    pause(time_resolution);  % Smooth motion
end


%%

% Define symbolic variables
syms theta1 theta2 d3 theta4 real
q = [theta1; theta2; d3; theta4];  % Joint variables

% Forward kinematics and Jacobian
T_fk = Bakel_Scara.fkine(q);  % Forward kinematics
J = Bakel_Scara.jacob0(q);    % Geometric Jacobian

% Define desired trajectory in task space
num_points = 100;
time_step = 0.001;  % Integration time (1ms)
t = linspace(0, num_points * time_step, num_points);

% Generate a linear trajectory in Cartesian space
p_start = [0.5; 0.2; 0.5];  % Start position
p_end = [0.7; 0.5; 0.6];    % End position
p_traj = [linspace(p_start(1), p_end(1), num_points);
          linspace(p_start(2), p_end(2), num_points);
          linspace(p_start(3), p_end(3), num_points)];

% Initialize joint variables
q_current = [0; 0; 0.4; 0];  % Initial joint configuration
q_history = zeros(4, num_points);  % To store joint positions

% Loop through the trajectory
for i = 1:num_points
    % Desired end-effector velocity (linear only)
    if i < num_points
        v_desired = (p_traj(:, i+1) - p_traj(:, i)) / time_step;  % Linear velocity
    else
        v_desired = [0; 0; 0];  % Stop at the last point
    end

    % Compute joint velocities using Jacobian Inverse
    J_current = double(subs(J, q, q_current));  % Evaluate Jacobian numerically
    q_dot = pinv(J_current(1:3, :)) * v_desired;  % Solve for joint velocities

    % Update joint positions using Euler integration
    q_current = q_current + q_dot * time_step;

    % Store joint positions
    q_history(:, i) = q_current;

    % Plot the robot
    Bakel_Scara.plot(q_current');
    pause(time_step);  % Pause to simulate real-time motion
end


