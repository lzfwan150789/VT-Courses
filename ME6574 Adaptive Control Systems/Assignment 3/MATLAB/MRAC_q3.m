function [xdot] = MRAC_q3(t,x)

global A B C Cr Ar Br gamma Lambda l K_star a0

xp = [x(1) x(2)]'; xr = [x(3) x(4)]'; 
w1 = x(5); w2 = x(6);
theta = x(7:10);
w_bar = x(11:14);

r = 8*sin(2*t); % reference
yp = C*xp;
yr = Cr*xr;
e= yp-yr; % error
w = [w1; w2; yp; r];

dxr = Ar*xr+Br*r; % reference system

dw_bar = w - a0*w_bar; % dphi = w - a0*phi;
dtheta = -sign(K_star)*gamma*w_bar*e;

u = theta'*w - sign(K_star)*gamma*w_bar.'*w_bar*e;
dxp = A*xp+B*u; % plant
dw1 = Lambda*w1 + l*u;
dw2 = Lambda*w2 + l*yp;
xdot = [dxp; dxr; dw1; dw2; dtheta; dw_bar];