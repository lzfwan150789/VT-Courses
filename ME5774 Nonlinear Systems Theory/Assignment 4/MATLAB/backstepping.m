%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 5774: Nonlinear Systems Theory     
%% Assignment 4 - Problem 1 
%% Backstepping - ODE function
%% #### -------------------------------------------------------------- #### 
function xdot = backstepping(t,x)
%% Paramaters
a= 2 ;
b= 3 ;
K=[1 0 10 0 100 0];

%% Calcullations
f=2*x(1)*tanh(x(1));
ksi_G=x(2)*tanh(x(1));
f_a=3*x(1)*x(2);
G_a=1/(1+x(2)^2);
W=x(1)^3*tanh(x(1));

%% ODEs
for i=1:2:5
    xdot(i)= a*x(i)*tanh(x(i))+x(i+1)*tanh(x(i));
    xdot(i+1)= (-a-2*x(i))*(a*x(i)*tanh(x(i))+x(i+1)*tanh(x(i)))...
        -(x(i)*tanh(x(i)))- K(i)*(x(i+1)+a*x(i)+x(i)^2);  % -a-2*
end
xdot=xdot';
end

%% Full representation - Appendix
% K1= 1 ;
% K2= 10 ;
% K3= 100 ;

% xdot(1)= a*x(1)*tanh(x(1))+x(2)*tanh(x(1));
% xdot(2)= (-a-2*x(1))*(a*x(1)*tanh(x(1))+x(2)*tanh(x(1)))...
%     -(x(1)*tanh(x(1)))- K1*(x(2)+a*x(1)+x(1)^2);
% 
% xdot(3)=  a*x(3)*tanh(x(3))+x(4)*tanh(x(3)); 
% xdot(4)= (-a-2*x(3))*(a*x(3)*tanh(x(3))+x(4)*tanh(x(3)))...
%     -(x(3)*tanh(x(3)))- K2*(x(4)+a*x(3)+x(3)^2);
% 
% xdot(5)=  a*x(5)*tanh(x(5))+x(6)*tanh(x(5)); 
% xdot(6)= (-a-2*x(5))*(a*x(5)*tanh(x(5))+x(6)*tanh(x(5)))...
%     -(x(5)*tanh(x(5)))- K3*(x(6)+a*x(5)+x(5)^2);