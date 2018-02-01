%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 5774: Nonlinear Systems Theory     
%% Assignment 5 - Problem 1 
%% Inverse variable transfomration
%% #### -------------------------------------------------------------- ####
clear all;
close all;
clc;

%% Polynomial solutions 
n=input('Select the polynomial order: '); % 2 or 3

switch n
    case 3
        syms z2 z1 z3
        p=[1 0 1 -2*z2+z1-z3-2]; % Equation: x^3+x-2*z2+z1-z3-2 == 0
        r=roots(p);
        x=simplify(r)      
    case 2
        syms x z2 z4 z3
        eqn=x^2+2*x-z2-2*z4-z3^2 == 0 % Equation
        solx =solve(eqn,x);
        x=simplify(solx)
end