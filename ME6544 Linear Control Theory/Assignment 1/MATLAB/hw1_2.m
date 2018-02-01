%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory     
%% Assignment 1 - Problem 1.2
%% Main - Van der Pol Oscillator
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;
%% Global Variables
global mu;
global A;
global omega;

%% Solve ODE
t_int = [0 200]; 
x0 = [0 .5 0 .5]'; 
tic;
[t,x]= ode23(@vdp, t_int, x0);
toc;

%% Figures
Ln={'-','-','--','--','-.','-.', ':', ':', '--', '--' };
figure (1) % Phase plot
for i=1:2:4
    plot( x(:,i),x(:,i+1),Ln{i}, 'LineWidth',2);	
    hold on;
end
set(gca,'FontSize',16);
grid on;
xlabel('State x_1');ylabel('State x_2');
legend('\mu=6, A=0','\mu=8.53, A=1.2');

figure (2) % States plot
for i=1:4
    plot(t,x(:,i), Ln{i}, 'LineWidth',2);
    hold on;
end
set(gca,'FontSize',16);
grid on;
xlabel('Time [s]');ylabel('States');
legend('x_1 (\mu=6, A=0)','x_2 (\mu=6, A=0)','x_1 (\mu=8.53, A=1.2)',...
    'x_2 (\mu=8.53, A=1.2)');