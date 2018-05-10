%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6574: Adaptive Control Systems   
%% Assignment 4 - Direct MRAC with Deadzone - Second order system
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% 
global A B C Ar Br Cr gamma Lambda l K_star a0 gamma_theta m_lb m_ub bl_lb bl_ub 
global br_lb br_ub m br bl b1_lb b1_ub b2_lb b2_ub theta_a
% System paramters
Ji = 0.0026;
Bi = 0.00057;
Li = 0.0045;
Ri = 0.5;
Ki = 0.56;
m_lb = .2;
m_ub = 5;
br_lb = 0;
br_ub = 10;
bl_lb = -10;
bl_ub = 0;
b1_lb = m_lb*br_lb;
b1_ub = m_ub*br_ub;
b2_lb = m_lb*bl_lb;
b2_ub = m_ub*bl_ub;
m = 2;
br = .8;
bl = -1.2;

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
gamma = .01;
gamma_theta = .05;
Lambda = -5;
l = 15;
K_star = Kp/Kr;
a0 = 2;

% Deadzone parameters
b1 = br*m;
b2 = bl*m;
theta_a = [m;b1;m;b2];

% ODE
Xp0 = [5 1]';
Xr0 = [2 3]';
w0 = ones(2,1);
theta0 = ones(4,1);
w_bar0 = ones(4,1);
theta_a_hat0 = [4 5 5 -1]';
x0 = [Xp0;Xr0;w0;theta0;w_bar0;theta_a_hat0 ];
t0 = [0 60];
options = odeset('OutputFcn',@odeplot);
[t,x]=ode45(@MRACDeadzone_q3, t0, x0,options); % , options

e1 =  x(:,1) - x(:,3); % Reconstruct error xp-xr
e2 =  x(:,2) - x(:,4); % Reconstruct error xp-xr
b_r = x(:,end-2)./x(:,end-3);
b_l = x(:,end)./x(:,end-1);

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

figure (3)
plot(t, e1, 'Linewidth', 2, 'Color', 'b');hold on;
plot(t, e2, 'Linewidth', 2, 'Color', 'g');hold on;
set(gca,'FontSize',26);hold on;
grid on;ylabel('Tracking Error');xlabel('Time [s]');

figure (4)
plot(t, x(:,end-3), 'Linewidth', 2, 'Color', 'b');hold on;
plot(t, b_r, 'Linewidth', 2, 'Color', 'm');hold on;
plot(t, x(:,end-1), 'Linewidth', 2, 'Color', 'g');hold on;
plot(t,  b_l, 'Linewidth', 2, 'Color', 'y');hold on;
set(gca,'FontSize',26);hold on;
grid on;ylabel('Deadzone Parameters');xlabel('Time [s]');
legend({'$\hat{m}_{r}$','$\hat{b}_{r}$','$\hat{m}_{l}$','$\hat{b}_l$'},'Interpreter','latex')


