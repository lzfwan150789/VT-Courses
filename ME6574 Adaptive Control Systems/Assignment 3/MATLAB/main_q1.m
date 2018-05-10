%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6574: Adaptive Control Systems   
%% Assignment 3 - Direct MRAC - First order system
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% 
global a b ar br gamma_x gamma_r

% System paramters
Ji = 0.0026;
Bi = 0.00057;
Li = 0.0045;
Ri = 0.5;
Ki = 0.56;

% Reduced first order system
a = -Bi/Ji - (Ki^2)/ (Ji*Ri)
b = Ki/(Ji*Ri);
c = 1;
sys = ss(a,b,c,[])
sys = tf(sys)

% Reference system
ar = -200;
br = 350;
cr = 1;
sys = ss(ar,br,cr,[])
sys = tf(sys)

% Controller parameters
gamma_x = .005;
gamma_r = .005;

% ODE
x0 = zeros(4,1);
t0 = [0 40];
options = odeset('OutputFcn',@odeplot);
[t,x]=ode45(@sMRAC_q1, t0, x0); 
 
% Figures 
figure (1)
plot(t, x(:,1), 'Linewidth', 2, 'Color', 'b');hold on; 
plot(t, x(:,2), 'Linewidth', 2,  'Color', 'm');hold on; 
set(gca,'FontSize',26);hold on;
grid on;ylabel('States');xlabel('Time [s]');
legend('Plant','Reference')

figure (2)
plot(t, x(:,3), 'Linewidth', 2, 'Color', 'b');hold on; 
plot(t, x(:,4), 'Linewidth', 2, 'Color', 'm');hold on; 
set(gca,'FontSize',26);hold on;
grid on;ylabel('Gains');xlabel('Time [s]');
legend('K_x','K_r')

