clear all
close all
clc

%% Parameters
a1= 0;
a2 = 0.5;
a3 = 0.5;
a4 = 0.0;


d1 = 1;
d2 = 0.0;
d3 = 0.0;
d4 = 1;


alfa1 = 0;
alfa2 = 0;
alfa3 = 0; 
alfa4 = 0;


%% manip def
L(1) = Link([0, d1, a1, alfa1, 0],'standard');
L(2) = Link([0, d2, a2, alfa2, 0],'standard');
L(3) = Link([0, d3, a3, alfa3, 0],'standard');
L(4) = Link([0, d4, a4, alfa4, 0],'standard');


% desired joint angle position
q = [0, -pi/2, -pi/6, 0];

b_scara = SerialLink(L, 'name', 'b_scara');

%L(2).offset=0;
%L(2).qlim=[0 pi];
b_scara.display()
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
b_scara.plot(q)



