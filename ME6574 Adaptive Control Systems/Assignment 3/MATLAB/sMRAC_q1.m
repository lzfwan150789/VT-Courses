function [xdot] = sMRAC_q1(t,x)

global a b ar br gamma_x gamma_r

xp = x(1); xr = x(2); Kx = x(3); Kr = x(4); % augmented state

e= xp-xr; % error
r = 5.7*sin(2*t); % reference
u_star = Kx*xp+Kr*r; % controller

dxp = a*xp+b*u_star; % plant
dxr = ar*xr+br*r; % reference system
dKx = -gamma_x*sign(b)*e*xp; % Kx dynamics of controller
dKr = -gamma_r*sign(b)*e*r; % Kr dynamics of controller

xdot = [dxp; dxr; dKx; dKr];