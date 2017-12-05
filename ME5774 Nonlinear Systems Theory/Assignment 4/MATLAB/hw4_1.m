%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 5774: Nonlinear Systems Theory     
%% Assignment 4 - Problem 1 
%% Backstepping - Main
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% Solve ODE
t_int = [0 300]; 
x0 = [1 1 1 1 1 1]'; 
tic;
[t,x]= ode45(@backstepping, t_int, x0);
toc;

%% Figures
L={'-','--','--','--','-.'};
figure (1)
for i=1:2:5
    plot( x(:,i),x(:,i+1),L{i}, 'LineWidth',2);
    hold on;
end
grid on;
xlabel('State x_1');ylabel('State x_2');
legend('K=1', 'K=10', 'K=100');

figure (2)
for i=1:2
    plot(t,x(:,i), L{i}, 'LineWidth',2);
    hold on;
end
grid on;
xlabel('Time [s]');ylabel('States');
legend('K=1, x_1', 'K=1, x_2');

