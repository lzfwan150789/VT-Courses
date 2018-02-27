%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory   
%% Assignment 1 - Parameter Estimation
%% #### -------------------------------------------------------------- #### 
function Xdot = estimation(t,x)
global  lambda r mu m epsilon technique g lambda1 lambda0

M=m+mu*sin(0.05*t); % Mass function
dM=0.05*mu*0.05*cos(0.05*t); % Mass derivative

% Plant
A = [0 1; -2 -dM/M]; % State matrix
B = [0; 1/M]; % Input matrix
X = [x(1) x(2)]'; % States
u = 2*sin(t); % Input
dx = A*X+B*u; % State space 
y = x(1); % Output

P = [x(3) x(4) x(5); x(6) x(7) x(8); x(9) x(10) x(11)];%Least square states
Theta = [x(12) x(13) x(14)]'; % Theta states (we want to find this)

% Measurements
um = u + epsilon*sin(3*t);
ym = y + epsilon*sin(3*t);

% Transfer Function to State Space
X_tf2ss = [x(15) x(16)]';
A_tf2ss = [0 1; -lambda1 -lambda0];
B_tf2ss = [0 1]';
dX_tf2ss = A_tf2ss*X_tf2ss+B_tf2ss*um;

X_tf2ss2 = [x(17) x(18)]';
dX_tf2ss2 = A_tf2ss*X_tf2ss2+B_tf2ss*ym;

W(1) = X_tf2ss(1); % Input measurement filter output
W(2) = -X_tf2ss2(2); % Output measurement filter output 1
W(3) = -X_tf2ss2(1); % Output measurement filter output 2

W=[W(1) W(2) W(3)]'; % Measurements filter outputs

k=1;
for i=1:3
    for j=1:3
        dP(k) = lambda*P(i,j)-P(i,:)*W*W'*P(:,j); %  LS derivative
        k=k+1;
    end
end

Pr=reshape(P,[3,3])';

if technique==1
    G=g*ones(3,3); % Gain matrix
    dTheta=-G*W*(W'*Theta-ym); % Theta derivative least square estimation
else
    dTheta=-Pr*W*(W'*Theta-ym); % Theta derivative gradient descent estimation
end

Xdot=[dx;dP';dTheta;dX_tf2ss;dX_tf2ss2;dM];
end