%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6574: Adaptive Control Systems   
%% Assignment 1 - Parameter Estimation
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% Parameters
global lambda r mu m epsilon technique g lambda1 lambda0 Am Q P A

technique=4; % 1 for gradient descent, 2 for least-squares,  
             % 3 for ls w/forgettings factor % 4 for adaptive estimation
noise=3; % 1 for noise-free, 2 for measurement noise, 3 for drift mass,
         % 4 for drift mass and measurement noise
% Filter values
lambda0 = 12; 
lambda1 = 7; 
p=[1 lambda1 lambda0];
r=roots(p); % poles in the left plane at -3 and -4 (stable)

alpha = 1987; % initialize least square matrix P (large value)
m = 1; %mass

% Estimation Cases
switch technique
    case 1 % gradient descent
       lambda = 0; % forgetting factor 
       g=10; % Fixed gain
    case 2 % least-squares
       lambda = 0; % forgetting factor 
    case 3 % least-squares with forgetting factor
       lambda = .5; % forgetting factor
    case 4 % adaptive estimation
       Q = 2*eye (2);
       Am = [0 1; -1 -2]; % Stable state matrix
       P = lyap(Am',Q);
end

% Noise Inputs
switch noise
    case 1 % noise-free
        mu = 0; 
        epsilon = 0;
    case 2 % measurements noise
        mu = 0; 
        epsilon = 2;
    case 3 % drift mass
        mu = 3; 
        epsilon = 0;
    case 4 % drift mass and measurement noise
        mu = 3;
        epsilon = 2;
end

%% Ode 
if technique > 3
    Tf=50;
    x0=[1 3]';
    M0 = 2;
    Ahat0 = [10 -10 2 -5]';
    Bhat0 = [3 9]';
    xhat0 = [1 1]';
    x_init = [x0;Ahat0;Bhat0;xhat0;M0];
    t_int = [0 Tf];
    
    options = odeset('OutputFcn',@odeplot); % , 'OutputSel',[1 2] 
    [t,x]= ode45(@adaptiveEstimation, t_int, x_init, options);
    hold on;
else   
    % Initial conditions
    x0=[1 2]';
    p0=alpha*eye(3);
    P0=reshape(p0,[9,1]);
    Theta0=[4 1 2]';
    M0=2;
    x0_tf2ss= [0 0 0 0]'; % Set it to zero, otherwise blows up
    x_init=[x0;P0;Theta0;x0_tf2ss;M0];
    t_int = [0 20];

    options = odeset('OutputFcn',@odeplot); % , 'OutputSel',[1 2] 
    [t,x]= ode45(@estimation, t_int, x_init, options);
    hold on;
end


%% Plots

[tr,tc]=size(t);
for i=1:tr
    m_desired(i,1) = m;
end
[sn]=size(x0);

l=1;
for k=1:sn
    for j=1:sn
        for i=1:tr
            a_desired(i,l) = A(k,j);
        end
        l=l+1;
    end
end



if technique > 3
    figure (2)
    plot(t, 1./x(:,8), 'LineWidth',2);hold on;
    plot(t,m_desired,'--', 'LineWidth',2, 'Color', 'r');hold on;
    set(gca,'FontSize',20);hold on;
    grid on;ylabel('Estimation of m [kg]');xlabel('Time [s]');

    figure (3)
    plot(t, x(:,8), 'LineWidth',2);hold on;
    plot(t,m_desired,'--', 'LineWidth',2, 'Color', 'r');hold on;
    set(gca,'FontSize',20);hold on;
    grid on;ylabel('Estimation of 1/m [kg^{-1}]');xlabel('Time [s]');
% Drift mass: Estimation of$\dot{m}/m$ [$kg^{-1}$]', 'interpreter','latex'

    figure (4)
    Ln = {'-','--','.-',':'};
    plot(t, x(:,3), Ln{1},'Color','r', 'LineWidth',2);hold on;
    plot(t, a_desired(:,1), Ln{1},'Color','r', 'LineWidth',2);hold on;
    plot(t, x(:,4), Ln{2},'Color','b', 'LineWidth',2);hold on;
    plot(t, a_desired(:,3), Ln{2},'Color','b', 'LineWidth',2);hold on;
    plot(t, x(:,5), Ln{3},'Color','g', 'LineWidth',2);hold on;
    plot(t, a_desired(:,2), Ln{3},'Color','g', 'LineWidth',2);hold on;
    plot(t, x(:,6), Ln{4},'Color','c', 'LineWidth',2);hold on;
    plot(t, a_desired(:,4), Ln{4},'Color','c', 'LineWidth',2);hold on;
    set(gca,'FontSize',20);hold on;
    grid on;ylabel('Estimation of state matrix A');xlabel('Time [s]');
else
    figure (2)
    plot(t, 1./x(:,12), 'LineWidth',2);hold on;
    plot(t,m_desired,'--', 'LineWidth',2, 'Color', 'r');hold on;
    set(gca,'FontSize',20);hold on;
    grid on;ylabel('Estimation of m [kg]');xlabel('Time [s]');

    figure (3)
    plot(t, x(:,12), 'LineWidth',2);hold on;
    plot(t,m_desired,'--', 'LineWidth',2, 'Color', 'r');hold on;
    set(gca,'FontSize',20);hold on;
    grid on;ylabel('Estimation of 1/m [kg^{-1}]');xlabel('Time [s]');
% Drift mass: Estimation of$\dot{m}/m$ [$kg^{-1}$]', 'interpreter','latex'
end













