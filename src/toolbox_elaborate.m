clear all
clc
%write the values of your DH tables out 
theta_i = [0 0 0 0];  
d_i=  [1 0 0.25 0 ];
a_i =  [ 0.5 0.5 0 0 ];
alpha_i = [0 0 0 0];
for i = 1:1:4;
    L(i) = Link([theta_i(i),d_i(i),a_i(i),alpha_i(i),0],'standard'); 
    if i == 3;
         L(i) = Link([theta_i(i),d_i(i),a_i(i),alpha_i(i),1],'standard'); 
    else   
    end
end
%L(2).offset = pi/2;
L(3).qlim = [0.25 1];

rv3sd = SerialLink(L,'name','rv3sd')
Q = [0 0 0.75 0];
rv3sd.plot(Q)
rv3sd.teach()