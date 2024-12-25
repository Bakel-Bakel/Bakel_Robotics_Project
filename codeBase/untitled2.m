

% Define DH parameters for each link
L1 = Revolute('d', 1, 'a', 0, 'alpha', 0);         % Revolute joint (Link 1)
L2 = Revolute('d', 0, 'a', 0.5, 'alpha', 0);           % Revolute joint (Link 2)
L3 = Prismatic('theta', 0, 'a', 0.5, 'alpha', 0, 'qlim', [0.1, 1]); % Prismatic joint (Link 3)
L4 = Revolute('d', 0, 'a', 0, 'alpha', 0);           % Revolute joint (Link 4)

% Combine the links into a serial-link robot
rrpr_robot = SerialLink([L1 L2 L3 L4], 'name', 'RRPR Robot');

% Display the robot structure
rrpr_robot.display();
rrpr_robot.plot([pi/4, pi/6, 0.5, pi/3]);  % Specify d3 = 0.5 for the prismatic joint

