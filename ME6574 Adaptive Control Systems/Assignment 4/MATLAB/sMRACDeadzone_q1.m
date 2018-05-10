function [xdot] = sMRACDeadzone_q1(t,x)

global a b aref bref gamma_x gamma_r gamma_theta m_lb m_ub bl_lb bl_ub
global br_lb br_ub theta_a m br bl b1_lb b1_ub b2_lb b2_ub technique Kx Kr

if technique ==1
    xp = x(1); xr = x(2);% Kx = x(3); Kr = x(4); % augmented state
    mr_hat = x(3); b1_hat = x(4); ml_hat = x(5); b2_hat = x(6); % deadzone states
    mr_hat = ml_hat;
    theta_a_hat = x(3:6); % 4x1
    
else
    xp = x(1); xr = x(2); Kx = x(3); Kr = x(4); % augmented state
    mr_hat = x(5); b1_hat = x(6); ml_hat = x(7); b2_hat = x(8); % deadzone states
    mr_hat = ml_hat;
    theta_a_hat = x(5:8); % 4x1
end

e= xp-xr; % error
r = 5.7*sin(2*t); % reference
ud = Kx*xp+Kr*r; % controller

%% Inverse deadzone output
if ud > 0
    v = (ud + theta_a_hat(2))/theta_a_hat(1);
elseif ud < 0
    v = (ud + theta_a_hat(4))/theta_a_hat(3);
else
    v = 0;
end

%% Inverse indicator functions
% Estimated values
br_hat = theta_a_hat(2)/theta_a_hat(1);
if v < br_hat
    Sr_hat = 0;
else
    Sr_hat = 1;
end

bl_hat = theta_a_hat(4)/theta_a_hat(3);
if v > bl_hat
    Sl_hat = 0;
else
    Sl_hat = 1;
end

% Real values
if v < br
    Sr = 0;
else
    Sr = 1;
end

if v > bl
    Sl = 0;
else
    Sl = 1;
end

da = (Sr - Sr_hat)*m*(v-br) + (Sl - Sl_hat)*m*(v-bl); % disturbance

%% Projection algorithm
Wa = [-Sr_hat*v; Sr_hat; -Sl_hat*v; Sl_hat];

dtheta_a_hat = -gamma_theta*sign(b)*Wa*e; % Theta_a update law

% Lower bound m_r
if theta_a_hat(1) <= m_lb && sign(dtheta_a_hat(1)) < 0
    dtheta_a_hat(1) = 0;
else
    dtheta_a_hat(1) = -gamma_theta*sign(b)*Wa(1)*e;
end

% Upper bound m_r
if  theta_a_hat(1) >= m_ub && sign(dtheta_a_hat(1)) > 0
    dtheta_a_hat(1) = 0;
else
    dtheta_a_hat(1) = -gamma_theta*sign(b)*Wa(1)*e;
end

% Lower bound m_l
if theta_a_hat(3) <= m_lb && sign(dtheta_a_hat(3)) < 0
    dtheta_a_hat(3) = 0;
else
    dtheta_a_hat(3) = -gamma_theta*sign(b)*Wa(3)*e;
end

% Upper bound m_l
if  theta_a_hat(3) >= m_ub && sign(dtheta_a_hat(3)) > 0
    dtheta_a_hat(3) = 0;
else
    dtheta_a_hat(3) = -gamma_theta*sign(b)*Wa(3)*e;
end

% Lower bound b_r
dbr_hat = (theta_a_hat(2)*dtheta_a_hat(1)- dtheta_a_hat(2)*theta_a_hat(1))/(theta_a_hat(1)^2);
if br_hat <= br_lb && sign(dbr_hat) < 0 
   dtheta_a_hat(2) = dtheta_a_hat(1)*(theta_a_hat(2)/theta_a_hat(1));
else
   dtheta_a_hat(2) = -gamma_theta*sign(b)*Wa(2)*e;
end

% Upper bound b_r
if b1_hat >= b1_ub && sign(dtheta_a_hat(2)) > 0 
   dtheta_a_hat(2) = 0;
else
   dtheta_a_hat(2) = -gamma_theta*sign(b)*Wa(2)*e;
end

% Lower bound b_l
if b2_hat <= b2_lb && sign(dtheta_a_hat(4)) < 0 
   dtheta_a_hat(4) = 0;
else
   dtheta_a_hat(4) = -gamma_theta*sign(b)*Wa(4)*e;
end

% Upper bound b_l
if b2_hat >= b2_ub && sign(dtheta_a_hat(4)) > 0 
   dtheta_a_hat(4) = 0;
else
   dtheta_a_hat(4) = -gamma_theta*sign(b)*Wa(4)*e;
end

phi = theta_a_hat - theta_a; % Define phi

u = ud + phi'*Wa + da; % Deadzone control
dxp = a*xp+b*u; % plant
dxr = aref*xr+bref*r; % reference system
if technique == 2
    dKx = -gamma_x*sign(b)*e*xp; % Kx dynamics of controller
    dKr = -gamma_r*sign(b)*e*r; % Kr dynamics of controller
    xdot = [dxp; dxr; dKx; dKr; dtheta_a_hat];
else
    xdot = [dxp; dxr; dtheta_a_hat];
end

