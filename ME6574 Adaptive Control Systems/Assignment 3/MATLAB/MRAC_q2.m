function [xdot] = MRAC_q2(t,x)

global A B Ar Br gamma_x gamma_r

xp = [x(1) x(2)]'; xr = [x(3) x(4)]'; 
Kx = x(5); Kr = x(6); % augmented state

e= xp(1)-xr(1); % error
r = 8*sin(2*t); % reference
u = Kx*xp(1)+Kr*r; % controller

dxp = A*xp+B*u; % plant
dxr = Ar*xr+Br*r; % reference system
dKx = -gamma_x*e*xp(1); % Kx dynamics of controller
dKr = -gamma_r*e*r; % Kr dynamics of controller

xdot = [dxp; dxr; dKx; dKr];