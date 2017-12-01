%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 5774: Nonlinear Systems Theory     
%% Assignment 4 - Problem 2 
%% Sliding Mode Control (SMC) - Main
%% #### -------------------------------------------------------------- ####
close all;
clear all;
clc;

%% Solve ODE
t_int = [0 20]; 
x0 = ones(8,1); 
tic;
[t,x]= ode45(@smc, t_int, x0);
toc;

%% Figures
L={'-','--','--','--','-.','-.',':',':'};
figure (1)
for i=1:2:7
    plot( x(:,i),x(:,i+1),L{i}, 'LineWidth',2);
    hold on;
end
grid on;
xlabel('State x_2');ylabel('State x_1');
legend('sign', 'sat =1', 'sat =10', 'sat =0.1');

figure (2)
for i=1:2
    plot(t,x(:,i), L{i}, 'LineWidth',2);
    hold on;
end
grid on;
xlabel('Time [s]');ylabel('States');
legend('x_1','x_2');


