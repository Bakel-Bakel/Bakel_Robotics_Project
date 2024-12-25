clear all
close all
clc

%% Parameters
a1= 0.095;
a2 = 0.245;
a3 = 0.135;
a4 = 0.0;
a5 = 0.0;
a6 = 0.0;

d1 = 0.350;
d2 = 0.0;
d3 = 0.0;
d4 = 0.270;
d5 = 0.0;
d6 = 0.085;

alfa1 = -pi/2;
alfa2 = 0;
alfa3 = pi/2; 
alfa4 = -pi/2;
alfa5= pi/2;
alfa6= 0.0;

%% manip def
L(1) = Link([0, d1, a1, alfa1, 0],'standard');
L(2) = Link([0, d2, a2, alfa2, 0],'standard');
L(3) = Link([0, d3, a3, alfa3, 0],'standard');
L(4) = Link([0, d4, a4, alfa4, 0],'standard');
L(5) = Link([0, d5, a5, alfa5, 0],'standard');
L(6) = Link([0, d6, a6, alfa6, 0],'standard');

% desired joint angle position
q = [0, -pi/2, -pi/6, 0, pi/3, 0];

rv3sd = SerialLink(L, 'name', 'rv3sd');

L(2).offset=pi/2;
L(2).qlim=[0 pi];

% x = 1;
% y = 1;
% z = 1;
% Ttool = transl([x,y,z]);
% rv3sd.tool = Ttool;
% x = 0;
% y = 0;
% z = 0;
% Tbase = transl([x,y,z]);
% rv3sd.tool = Tbase;
figure
rv3sd.plot(q)

figure
q = [pi/2,-pi/6, -pi/6, 0,pi,0];
rv3sd.plot(q)

