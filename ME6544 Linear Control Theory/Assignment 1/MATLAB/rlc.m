%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory   
%% Assignment 1 - Problem 1.1.2 
%% RLC circuit
%% #### -------------------------------------------------------------- #### 
function xdot = rlc(t,x)
global R;
global L;
global C;
%% Paramaters
R=[.1 .1 .5 .5 3 3 20 20]';
L=[1 1 .3 .3 6 6 15 15]';
C=[10 10 5 5 1 1 2 2]';
u=sin(t);

%% ODEs
for i=1:2:8
    xdot(i)= (-1/C(i))*x(i+1)+(1/C(i))*u;
    xdot(i+1)= (1/L(i))*x(i)-(R(i)/L(i))*x(i+1);  
end 
xdot=xdot';
end