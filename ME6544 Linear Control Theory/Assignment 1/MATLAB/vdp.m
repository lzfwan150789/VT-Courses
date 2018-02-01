%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory   
%% Assignment 1 - Problem 1.2 
%% Van Der Pol Ocillator
%% #### -------------------------------------------------------------- #### 
function xdot = vdp(t,x)
global mu;
global A;
global omega;
%% Paramaters
mu=[6 6 8.53 8.53]';
A=[0 0 1.2 1.2]';
omega=(2*pi)/10;
u=A*sin(omega*t);

%% ODEs
for i=1:2:4
    xdot(i)= x(i+1);
    xdot(i+1)= mu(i)*(1-x(i)^2)*x(i+1)-x(i)+u(i);  
end 
xdot=xdot';
end