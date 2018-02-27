%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory   
%% Assignment 1 - Parameter Estimation
%% #### -------------------------------------------------------------- #### 
function Xdot = adaptiveEstimation(t,x)
global  mu m epsilon Am P um A

M=m+mu*sin(0.05*t); % Mass function
dM=0.05*mu*0.05*cos(0.05*t); % Mass derivative

% Plant
A = [0 1; -2 -dM/M]; % State matrix with some damping
B = [0; 1/M]; % Input matrix
X = [x(1) x(2)]'; % States
u = 2*sin(t); % Input
dx = A*X+B*u; % State space 
y = x(1); % Output
Ahat = [x(3) x(4);x(5) x(6)]'; 
Bhat = [x(7) x(8)]';

% Measurements
um = u + epsilon*sin(3*t);
ym = y + epsilon*sin(3*t);
Xm = [ym x(2)]';

% Xhat = [x(10) x(11)]'; % State 
Xhat = [x(9) x(10)]'; % State 
em = Xhat-Xm; % Define error

dAhat = -P*em*Xm'; % State matrix estimation
dAhat_v = reshape(dAhat,[4,1]);
dBhat = -P*em*um; % Input matrix estimation

dXhat = Am*Xhat+(Ahat-Am)*Xm+Bhat*um; % Estimator

Xdot=[dx;dAhat_v;dBhat;dXhat;dM];
end