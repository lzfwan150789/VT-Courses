function [xdot] = MRACDeadzone_q3(t,x)

global A B C Cr Ar Br gamma Lambda l K_star a0 gamma_theta m_lb m_ub 
global br_lb br_ub theta_a m br bl b1_lb b1_ub b2_lb b2_ub bl_lb bl_ub 

xp = [x(1) x(2)]'; xr = [x(3) x(4)]'; 
w1 = x(5); w2 = x(6);
theta = x(7:10);
w_bar = x(11:14);
mr_hat = x(15); b1_hat = x(16); ml_hat = x(17); b2_hat = x(18); % deadzone states
mr_hat = ml_hat;
theta_a_hat = x(15:18); % 4x1

r = 8*sin(2*t); % reference
yp = C*xp;
yr = Cr*xr;
e= yp-yr; % error
w = [w1; w2; yp; r];

dxr = Ar*xr+Br*r; % reference system

dw_bar = w - a0*w_bar; % dphi = w - a0*phi;
dtheta = -sign(K_star)*gamma*w_bar*e;

ud = theta'*w - sign(K_star)*gamma*w_bar.'*w_bar*e;

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

dtheta_a_hat = -gamma_theta*1*Wa*e; % Theta_a update law

% Lower bound m_r
if theta_a_hat(1) <= m_lb && sign(dtheta_a_hat(1)) < 0
    dtheta_a_hat(1) = 0;
else
    dtheta_a_hat(1) = -gamma_theta*1*Wa(1)*e;
end

% Upper bound m_r
if  theta_a_hat(1) >= m_ub && sign(dtheta_a_hat(1)) > 0
    dtheta_a_hat(1) = 0;
else
    dtheta_a_hat(1) = -gamma_theta*1*Wa(1)*e;
end

% Lower bound m_l
if theta_a_hat(3) <= m_lb && sign(dtheta_a_hat(3)) < 0
    dtheta_a_hat(3) = 0;
else
    dtheta_a_hat(3) = -gamma_theta*1*Wa(3)*e;
end

% Upper bound m_l
if  theta_a_hat(3) >= m_ub && sign(dtheta_a_hat(3)) > 0
    dtheta_a_hat(3) = 0;
else
    dtheta_a_hat(3) = -gamma_theta*1*Wa(3)*e;
end

% Lower bound b_r
dbr_hat = (theta_a_hat(2)*dtheta_a_hat(1)- dtheta_a_hat(2)*theta_a_hat(1))/(theta_a_hat(1)^2);
if br_hat <= br_lb && sign(dbr_hat) < 0 
   dtheta_a_hat(2) = dtheta_a_hat(1)*(theta_a_hat(2)/theta_a_hat(1));
else
   dtheta_a_hat(2) = -gamma_theta*1*Wa(2)*e;
end

% Upper bound b_r
if b1_hat >= b1_ub && sign(dtheta_a_hat(2)) > 0 
   dtheta_a_hat(2) = 0;
else
   dtheta_a_hat(2) = -gamma_theta*1*Wa(2)*e;
end

% Lower bound b_l
if b2_hat <= b2_lb && sign(dtheta_a_hat(4)) < 0 
   dtheta_a_hat(4) = 0;
else
   dtheta_a_hat(4) = -gamma_theta*1*Wa(4)*e;
end

% Upper bound b_l
if b2_hat >= b2_ub && sign(dtheta_a_hat(4)) > 0 
   dtheta_a_hat(4) = 0;
else
   dtheta_a_hat(4) = -gamma_theta*1*Wa(4)*e;
end

phi = theta_a_hat - theta_a; % Define phi

u = ud + phi'*Wa + da; % Deadzone control

dxp = A*xp+B*u; % plant
dw1 = Lambda*w1 + l*ud;
dw2 = Lambda*w2 + l*yp;
xdot = [dxp; dxr; dw1; dw2; dtheta; dw_bar; dtheta_a_hat];