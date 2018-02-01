%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory     
%% Assignment 1 - Problem 1.1.2 
%% Main - RLC circuit
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;
%% Global Variables
global R;

%% Solve ODE
t_int = [0 200]; 
x0 = zeros(8,1); 
tic;
[t,x]= ode23(@rlc, t_int, x0);
toc;
j=1;
for i=2:2:8
    y(:,j)=R(i)*x(:,i); % Output function
    j=j+1;
end

%% Figures
Ln={'-','-','--','--','-.','-.', ':', ':', '--', '--' };
figure (1) % Phase plot
for i=1:2:8
    plot( x(:,i),x(:,i+1),Ln{i}, 'LineWidth',2);	
    hold on;
end
set(gca,'FontSize',16);
grid on;
xlabel('State x_1');ylabel('State x_2');
legend('R=.1, L=1, C=10','R=.5, L=.3, C=5','R=3, L=6, C=1','R=20, L=15, C=2');

figure (2) % States plot
for i=1:2:7
    plot(t,x(:,i), Ln{i}, 'LineWidth',2);
    hold on;
end
set(gca,'FontSize',16);
grid on;
xlabel('Time [s]');ylabel('State x_1');
% xlabel('Time [s]', 'FontSize', 16);ylabel('States', 'FontSize', 16);
legend('x_1 (R=.1, L=1, C=10)','x_1 (R=.5, L=.3, C=5)',...
    'x_1 (R=3, L=6, C=1)',...
    'x_1 (R=20, L=15, C=2)');

figure (3) % States plot
for i=2:2:8
    plot(t,x(:,i), Ln{i}, 'LineWidth',2);
    hold on;
end
set(gca,'FontSize',16);
grid on;
xlabel('Time [s]');ylabel('State x_2');
legend('x_2 (R=.1, L=1, C=10)',...
    'x_2 (R=.5, L=.3, C=5)','x_2 (R=3, L=6, C=1)',...
    'x_2 (R=20, L=15, C=2)');

figure (4) % Output plot
j=1;
for i=1:4
    plot( t,y(:,i),Ln{j}, 'LineWidth',2);
    hold on;
    j=j+1;
end
set(gca,'FontSize',16);
grid on;
xlabel('Time [s]');ylabel('Output y');
legend('R=.1, L=1, C=10','R=.5, L=.3, C=5','R=3, L=6, C=1','R=20, L=15, C=2');