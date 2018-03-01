%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory     
%% Assignment 2 
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% Problem 1
syms k s
A1 = [-1 0;k -2];
B1 = [1 1]';
C1 = [0 1];
Delta1 = det(s*eye(2)-A1) % Characteristic equation
Phi1 = inv(s*eye(2)-A1) % Transition matrix

%% Problem 2
A2 = [1 -1;-9 -5];
B2 = [1 0;1 1];
C2 = [2 2; -4 -1];
Adj2 = adjoint(s*eye(2)-A2)
Delta2 = det(s*eye(2)-A2) % Characteristic equation
Phi2 = inv(s*eye(2)-A2) % Transition matrix
H2 = C2*Phi2*B2 % Transfer function
H2_short = (C2*Adj2*B2)/Delta2 % Transfer function
p2 = roots([1 4 -14]); % Poles
Co2 = ctrb(A2,B2);
unco2 = length(A2) - rank(Co2);
Ob2 = obsv(A2,C2);
unob2 = length(A2)-rank(Ob2);
OR_2 = [C2;C2*A2]*[B2 A2*B2] % Minimal realization check
rank(OR_2) % Minimal realization check

s = tf('s');
P2=[4*s-12 2*s-4; -5*s-6 -s+5];
z = tzero(P2) % Zeros Smith-McMiller

%% Problem 3
syms s t
A3 = [1 1;3 5];
B3 = [0 1]';
C3 = [1 0];
Adj3 = adjoint(s*eye(2)-A3)
Delta3 = det(s*eye(2)-A3) % Characteristic equation
Phi3 = inv(s*eye(2)-A3) % Transition matrix
ilt3_1 = ilaplace(Phi3)
H3 = C3*Phi3*B3 % Transfer function
H3_short = (C3*Adj3*B3)/Delta3 % Transfer function
p3 = roots([1 -6 2]) % Poles
sys = ss(A3,B3,C3,0);
ilt3_2 = ilaplace(H3) % Impulse response formula
ilt3_3 = ilaplace(H3/s) % Step response formula
figure (1)
impulse(sys); hold on;
figure (2)
step(sys); hold on;

%% Problem 4
A4 = [-1/(t+1) 0;-1/(t+1) 0];
eig(A4) % Eigenvalues (marginally stable)

%% Problem 5
A5 = [-1 exp(2*t);0 -1];
eig(A5) % Eigenvalues (unstable, am>gm)