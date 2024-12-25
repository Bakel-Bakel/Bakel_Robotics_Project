clc
clear
% Direct Kinematic

% Define desired end-effector position
x = 0.4830;
y = 0.8365;
z = 0.75;

% SCARA parameters
a1 = 0.5;
a2 = 0.5;
d0 = 1.0;

% Inverse kinematics
r = sqrt(x^2 + y^2);

% Compute theta2 using law of cosines
cos_theta2 = (r^2 - a1^2 - a2^2) / (2 * a1 * a2);
theta2 = atan2(sqrt(1 - cos_theta2^2), cos_theta2);

% Compute theta1
theta1 = atan2(y, x) - atan2(a2*sin(theta2), a1 + a2*cos(theta2));

% Compute d3
d3 = z-d0;

% Display results
theta1_deg = rad2deg(theta1);
theta2_deg = rad2deg(theta2);

disp(['Theta1: ', num2str(theta1_deg), ' degrees']);
disp(['Theta2: ', num2str(theta2_deg), ' degrees']);
disp(['D3: ', num2str(d3), ' meters']);
