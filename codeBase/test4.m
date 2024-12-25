% Define symbolic variables for joint values
syms theta1 theta2 d3 theta4 a1 a2 real 

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
