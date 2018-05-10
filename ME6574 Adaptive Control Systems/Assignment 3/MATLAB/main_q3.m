%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6574: Adaptive Control Systems   
%% Assignment 3 - Direct MRAC - Second order system
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% 
global A B C Ar Br Cr gamma Lambda l K_star a0 

% System paramters
Ji = 0.0026;
Bi = 0.00057;
Li = 0.0045;
Ri = 0.5;
Ki = 0.56;

% System
A = [-Bi/Ji Ki/Ji; -Ki/Li -Ri/Li];
B = [0 1/Li]';
C = [1 0];


Kp = 4.786e04;

% Reference system
Ar = [-.1 200; -120 -110];
Br = [0 150]';
Cr = [1 0];

Kr = 125; % needs to be changed if I change the plant

% Controller parameters
gamma = .05
Lambda = -4;
l = 1;
K_star = Kp/Kr;
a0 = 2;

% ODE
x0 = zeros(14,1);
t0 = [0 40];
options = odeset('OutputFcn',@odeplot);
[t,x]=ode23(@MRAC_q3, t0, x0); % , options

% Figures 
figure (1)
plot(t, x(:,1), 'Linewidth', 2, 'Color', 'b');hold on; 
plot(t, x(:,3), 'Linewidth', 2, 'Color', 'm');hold on; % 'LineWidth',2
set(gca,'FontSize',20);hold on;
grid on;ylabel('States');xlabel('Time [s]');
legend('Plant','Reference')

figure (2)
plot(t, x(:,7), 'Linewidth', 2, 'Color', 'b');hold on; 
plot(t, x(:,8), 'Linewidth', 2, 'Color', 'm');hold on; % 'LineWidth',2
plot(t, x(:,9), 'Linewidth', 2, 'Color', 'g');hold on; 
plot(t, x(:,10), 'Linewidth', 2, 'Color', 'c');hold on; % 'LineWidth',2
set(gca,'FontSize',20);hold on;
grid on;ylabel('Gains');xlabel('Time [s]');
legend('\theta_1','\theta_2','\theta_3','\theta_4')


