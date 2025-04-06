
clc
clear
format short

% Direct Kinematic parameters
theta1 =  45; % Joint angle theta1 (convert to radians)
theta2 = 30; % Joint angle theta2 (convert to radians)
d3 = -0.25;            % Prismatic joint displacement d3(0.25 to 1.00)
theta4 = 0;  % Joint angle theta4 (for rotation)##does not have any effect on End-Effector pos

a1 = 0.5; % Link 1 length in meters
a2 = 0.5; % Link 2 length in meters 
d0 = 1.0; % Base height in meters

% Transformation/Rotation matrices
T01 = [cosd(theta1), -sind(theta1), 0, a1*cosd(theta1);
       sind(theta1), cosd(theta1),  0, a1*sind(theta1);
       0,           0,            1, d0;
       0,           0,            0, 1];
   
T12 = [cosd(theta2), -sind(theta2), 0, a2*cosd(theta2);
       sind(theta2), cosd(theta2),  0, a2*sind(theta2);
       0,           0,            1, 0;
       0,           0,            0, 1];
   
T23 = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, d3;
       0, 0, 0, 1];
   
T34 = [cosd(theta4), -sind(theta4), 0, 0;
       sind(theta4), cosd(theta4),  0, 0;
       0,           0,              1, 0;
       0,           0,              0, 1];
   
% Transformation/Rotation  matrix from base to end-effector
T04 = T01 * T12 * T23 * T34;

% End-effector position
end_effector_position = T04(1:3, 4);

% Display results
disp('Results for Direct Kinematics Transformation matrix and End_Effector')
disp(T04)
disp('Rotation/Transformation Matrix')
disp(T04(1:3,1:3))
disp('Position of End-Effector'); % Display Name 
disp(end_effector_position); % Display end effector values 
