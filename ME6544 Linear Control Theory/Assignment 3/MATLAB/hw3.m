%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory     
%% Assignment 3 
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% Problem 1
% System 1
syms s
A1 = [0 1 0 0; 0 0 -1 0; 0 0 0 1; 0 0 10 0];
B1 = [0 0.1 0 -0.1]';
C1 = [1 0 0 0];
sys1 = ss(A1,B1,C1,0);
Adj1 = adjoint(s*eye(4)-A1);
Delta1 = det(s*eye(4)-A1) % Characteristic equation
Phi1 = inv(s*eye(4)-A1) % Transition matrix
H1 = C1*Phi1*B1 % Transfer function
H1_short = (C1*Adj1*B1)/Delta1 % Transfer function
Co1 = ctrb(A1,B1);
Unco1 = det(Co1)
unco1 = length(A1) - rank(Co1)
Ob1 = obsv(A1,C1);
Unob1 = det(Ob1)
unob1 = length(A1)-rank(Ob1)
P1 = roots([-1 0 +10 0 0]) % Poles
Z1 = roots([-1/10 0 9/10]) % Zeros
eig(A1) % Eigenvalues (unstable)


% System 2
A2 = [0 1 0 0; 0 0 -9 0; 0 0 0 1; -1 0 0 0];
B2 = [0 0 0 -0.1]';
C2 = [1 0 0 0];
sys2 = ss(A2,B2,C2,0);
Adj2 = adjoint(s*eye(4)-A2);
Delta2 = det(s*eye(4)-A2) % Characteristic equation
Phi2 = inv(s*eye(4)-A2) % Transition matrix
H2 = C2*Phi2*B2 % Transfer function
H2_short = (C2*Adj2*B2)/Delta2 % Transfer function
Co2 = ctrb(A2,B2);
Unco2 = det(Co2)
unco2 = length(A2) - rank(Co2)
Ob2 = obsv(A2,C2);
Unob2 = det(Ob2)
unob2 = length(A2)-rank(Ob2)
P2 = roots([1 0 0 0 -9]) % Poles
eig(A2) % Eigenvalues (unstable)

% System 3
A3 = [0 1 0 0; -1 -.2 1 .2; 0 0 0 1; 1 .2 -10 -2.1];
B3 = [0 1 0 0]';
C3 = [0 0 1 0];
sys3 = ss(A3,B3,C3,0);
Adj3 = adjoint(s*eye(4)-A3);
Delta3 = det(s*eye(4)-A3) % Characteristic equation
Phi3 = inv(s*eye(4)-A3) % Transition matrix
H3 = C3*Phi3*B3 % Transfer function
H3_short = (C3*Adj3*B3)/Delta3 % Transfer function
Co3 = ctrb(A3,B3);
Unco3 = det(Co3)
unco3 = length(A3) - rank(Co3)
Ob3 = obsv(A3,C3);
Unob3 = det(Ob3)
unob3 = length(A3)-rank(Ob3)
P3 = roots([1 2.3 569/50 3.7 9]) % Poles
Z3 = roots([1/5 1]) % Zeros
figure (1)
impulse(sys3); hold on;
figure (2)
step(sys3); hold on;
eig(A3) % Eigenvalues (AS for LTI ES too)


% System 4
A4 = [0 1 0 0; -1 -.1 1 .1; 0 0 0 1; 1 .1 -23 -.3];
B4 = [0 1 0 0]';
C4 = [1 0 0 0];
sys4 = ss(A4,B4,C4,0);
Adj4 = adjoint(s*eye(4)-A4);
Delta4 = det(s*eye(4)-A4) % Characteristic equation
Phi4 = inv(s*eye(4)-A4) % Transition matrix
H4 = C4*Phi4*B4 % Transfer function
H4_short = (C4*Adj4*B4)/Delta4 % Transfer function
Co4 = ctrb(A4,B4);
Unco4 = det(Co4)
unco4 = length(A4) - rank(Co4)
Ob4 = obsv(A4,C4);
Unob4 = det(Ob4)
unob4 = length(A4)-rank(Ob4)
P4 = roots([1 2/5 1201/50 12/5 22]) % Poles
Z4 = roots([1 .3 23]) % Zeros
figure (3)
impulse(sys4); hold on;
figure (4)
step(sys4); hold on;
eig(A4) % Eigenvalues (AS for LTI ES too)


%% Problem 2
% System 1
A5 = [-4 -2; 1 -1];
Q5 = eye(2);
P5 = lyap(A5',Q5) % AS for LTI ES too

% System 2
A6 = [0 1; 0 -4];
eig(A6) % marginally stable
% P6 = lyap(A6',Q5) % marginally stable

%% Problem 3
Den6 = [1 3];
Num6 = [1 -7 14 8];
H6 = tf(Den6,Num6)
[r,p,k] = residue(Num6,Den6)
H6c = canon(H6,'companion')
A6_c = H6c.A
B6_c = H6c.B
C6_c = H6c.C

A6_o = A6_c'
B6_o = B6_c'
C6_o = C6_c'