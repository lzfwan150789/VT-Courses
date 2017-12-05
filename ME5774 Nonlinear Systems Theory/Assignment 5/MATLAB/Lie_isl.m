%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 5774: Nonlinear Systems Theory     
%% Assignment 5 - Problem 1,2 
%% Lie bracket analysis for input-state linearizable
%% #### -------------------------------------------------------------- ####
clear all;
close all;
clc;

%% Problem 1
% Nonlinear System 
%
%       |-x1+x2      |   |0 |
%dx/dt= |-x2+x3+x3^3 | + |0 |u
%       |-x3         |   |1 |
%
syms x
v1 = [0 0 1]'; % g
v2 = [0 -1-3*x^2 -1]'; % [f,g]
v3 = [1+3*x^2 -6*x^2 -1]'; % [f,[f,g]]
v4 = [0 -6*x 0]'; % [g,[f,g]]
V1=[v1 v2 v3]
V1_d=det(V1)
D1=[v1 v2 v4]

Rv1 = rank(V1)
Rd1 = rank(D1)

%% Problem 2
% Nonlinear System 
%
%       |tan(x2)/(1+x1^2) |   |0      |
%dx/dt= |sin(x3)          | + |0      |u
%       |1+x2^2           |   |1+x2^2 |
%
syms x1 x2 x3
v5 = [0 0 1+x2^2]'; % g
v6 = [0 -cos(x3)*(1+x2^2) 2*x2*sin(x3)]'; % [f,g]
v71= -(cos(x3)*(1+x2^2)^2)/(cos(x1)*(1+x1^2)^2);
v72= sin(x3)*(-2*x2*cos(x3)+(1+x2^2)^2-2*x2*cos(x3));
v73= 2*(sin(x3))^2+4*x2*cos(x3)*(1+x2^2);
v7 = [v71 v72 v73]'; % [f,[f,g]]
v8 = [0 sin(x3)*(1+x2^2)^2 4*x2*cos(x3)*(1+x2^2)^2]'; % [g,[f,g]]
V2=[v5 v6 v7]
V2_d=det(V2)
D2=[v5 v6 v8]

Rv2 = rank(V2)
Rd2 = rank(D2)