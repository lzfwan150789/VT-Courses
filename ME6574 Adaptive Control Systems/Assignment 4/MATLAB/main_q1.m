%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6574: Adaptive Control Systems   
%% Assignment 4 - Direct MRAC with Deadzone - First order system
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% Parameters
global a b aref bref gamma_x gamma_r gamma_theta m_lb m_ub bl_lb bl_ub 
global br_lb br_ub theta_a m br bl b1_lb b1_ub b2_lb b2_ub technique Kx Kr

technique=1; % 1 known motor/load, 2 unknown motor/load

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


% Reduced first order system
a = -Bi/Ji - (Ki^2)/ (Ji*Ri);
b = Ki/(Ji*Ri);
c = 1;
sys = ss(a,b,c,[]);
sys = tf(sys);

% Reference system
aref = -200;
bref = 350;
cr = 1;
sys = ss(aref,bref,cr,[]);
sys = tf(sys);

switch technique
    case 1 % known motor/load
        Kx = (aref - a)/b;
        Kr = bref/b;
        % Controller parameters
        gamma_theta = 50;
    case 2 % unknown motor/load
        % Controller parameters
        gamma_theta = 5;
        gamma_x = .6;
        gamma_r = .6;
end

% Deadzone parameters
b1 = br*m;
b2 = bl*m;
theta_a = [m;b1;m;b2];

%% ODE and figures

% ODE
if technique == 1
    Xp0 = 5;
    Xr0 = 2;
    x0 = [Xp0;Xr0; 4; 5; 5; -1 ];
    t0 = [0 10];
    options = odeset('OutputFcn',@odeplot);
    [t,x]=ode45(@sMRACDeadzone_q1, t0, x0, options);
    
else
    Xp0 = 5;
    Xr0 = 2;
    Kx0 = 1.5;
    Kr0 = 1;
    x0 = [Xp0;Xr0;Kx0;Kr0; 3; 5; 5; -1 ];
    t0 = [0 20];
    options = odeset('OutputFcn',@odeplot);
    [t,x]=ode45(@sMRAC_q1, t0, x0, options);
end

e =  x(:,1) - x(:,2); % Reconstruct error xp-xr
b_r = x(:,end-2)./x(:,end-3);
b_l = x(:,end)./x(:,end-1);
 
% Figures 
if technique ==1
    figure (1)
    plot(t, x(:,1), 'Linewidth', 2, 'Color', 'b');hold on;
    plot(t, x(:,2), 'Linewidth', 2,  'Color', 'm');hold on;
    set(gca,'FontSize',26);hold on;
    grid on;ylabel('States');xlabel('Time [s]');
    legend('Plant','Reference')
    
    figure (2)
    plot(t, e, 'Linewidth', 2, 'Color', 'b');hold on;
    set(gca,'FontSize',26);hold on;
    grid on;ylabel('Tracking Error');xlabel('Time [s]');
    
    figure (3)
    plot(t, x(:,end-3), 'Linewidth', 2, 'Color', 'b');hold on;
    plot(t, b_r, 'Linewidth', 2, 'Color', 'm');hold on;
    plot(t, x(:,end-1), 'Linewidth', 2, 'Color', 'g');hold on;
    plot(t,  b_l, 'Linewidth', 2, 'Color', 'y');hold on;
    set(gca,'FontSize',26);hold on;
    grid on;ylabel('Deadzone Parameters');xlabel('Time [s]');
    legend({'$\hat{m}_{r}$','$\hat{b}_{r}$','$\hat{m}_{l}$','$\hat{b}_l$'},'Interpreter','latex')
else
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
    
    figure (3)
    plot(t, e, 'Linewidth', 2, 'Color', 'b');hold on;
    set(gca,'FontSize',26);hold on;
    grid on;ylabel('Tracking Error');xlabel('Time [s]');
    
    figure (4)
    plot(t, x(:,5), 'Linewidth', 2, 'Color', 'b');hold on;
    plot(t, b_r, 'Linewidth', 2, 'Color', 'm');hold on;
    plot(t, x(:,7), 'Linewidth', 2, 'Color', 'g');hold on;
    plot(t,  b_l, 'Linewidth', 2, 'Color', 'y');hold on;
    set(gca,'FontSize',26);hold on;
    grid on;ylabel('Deadzone Parameters');xlabel('Time [s]');
    legend({'$\hat{m}_{r}$','$\hat{b}_{r}$','$\hat{m}_{l}$','$\hat{b}_l$'},'Interpreter','latex')
end




