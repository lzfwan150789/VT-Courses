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
global A B Ar Br gamma_x gamma_r

% System paramters
Ji = 0.0026;
Bi = 0.00057;
Li = 0.0045;
Ri = 0.5;
Ki = 0.56;

% System
A = [-Bi/Ji Ki/Ji; -Ki/Li -Ri/Li]
B = [0 1/Li]'
C = [1 0]
sys = ss(A,B,C,[]);
sys = tf(sys)

% Reference system
Ar = [-.1 200; -120 -110];
Br = [0 150]';
Cr = [1 0];
sys = ss(Ar, Br, Cr, [])
sys = tf(sys)

% Controller parameters
gamma_x = .001;
gamma_r = .001;

% ODE
x0 = zeros(6,1);
t0 = [0 40];
options = odeset('OutputFcn',@odeplot);
[t,x]=ode45(@MRAC_q2, t0, x0); 


% Figures 
figure (1)
plot(t, x(:,1),'Linewidth', 2, 'Color', 'b');hold on; 
plot(t, x(:,3),'Linewidth', 2, 'Color', 'm');hold on; 
set(gca,'FontSize',26);hold on;
grid on;ylabel('States');xlabel('Time [s]');
legend('Plant','Reference')

figure (2)
plot(t, x(:,5),'Linewidth', 2, 'Color', 'b');hold on; 
plot(t, x(:,6),'Linewidth', 2, 'Color', 'm');hold on;
set(gca,'FontSize',26);hold on;
grid on;ylabel('Gains');xlabel('Time [s]');
legend('K_x','K_r')


