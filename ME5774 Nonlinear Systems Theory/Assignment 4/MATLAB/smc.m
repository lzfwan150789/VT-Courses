%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 5774: Nonlinear Systems Theory     
%% Assignment 4 - Problem 2 
%% Sliding Mode Control (SMC) - ODE function
%% #### -------------------------------------------------------------- ####
function xdot = smc(t,x)
a= 2.5 ; % model parameter
b= 3.3 ; % model parameter

alpha=3; % control parameter
c=10; % control parameter
K=0; % control parameter
epsilon=1;
epsilon1=10;
epsilon2=.1;
%phi=-2*x(1)-alpha*abs(x(1));
%%
z=x(2)+2*x(1)+alpha*abs(x(1));
rho=abs(x(1))*abs(x(2))+(2+alpha)*abs(x(1)); %x(1)^2; 
beta=rho+c;
v=-(beta/(1-K))*sign(z);

f=(2*x(1)+x(2))*tanh(x(1));
d_1=(a-2)*x(1)*tanh(x(1));

f_a=3*x(1)*x(2);
G_a=1/(1+x(2)^2);
d_2=x(1)*x(2)*(1+x(2)^2)*(b-3);

del_phi=-2-alpha*sign(x(1));

u_eq=(1/G_a)*(-f_a+(del_phi)*f);
u= u_eq+(1/G_a)*v;

xdot(1)= f + d_1;
xdot(2)= f_a+G_a*(u+d_2);
%%
z_1=x(4)+2*x(3)+alpha*abs(x(3));
rho_1=abs(x(3))*abs(x(4))+(2+alpha)*abs(x(3)); %x(1)^2; 
beta_1=rho_1+c;

Z=z_1/epsilon;
if abs(Z)>1
    y=sign(Z);
else 
    y=Z;
end
v_1=-(beta_1/(1-K))*y;

f_1=(2*x(3)+x(4))*tanh(x(3));
d_11=(a-2)*x(3)*tanh(x(3));

f_a1=3*x(3)*x(4);
G_a1=1/(1+x(4)^2);
d_21=x(3)*x(4)*(1+x(4)^2)*(b-3);

del_phi_1=-2-alpha*y;

u_eq1=(1/G_a1)*(-f_a1+(del_phi_1)*f_1);
u_1=u_eq1+(1/G_a1)*v_1;

xdot(3)= f_1 + d_11;
xdot(4)= f_a1+G_a1*(u_1+d_21);
%%

z_2=x(6)+2*x(5)+alpha*abs(x(5));
rho_2=abs(x(5))*abs(x(6))+(2+alpha)*abs(x(5)); %x(1)^2; 
beta_2=rho_2+c;
Z1=z_2/epsilon1;
if abs(Z1)>1
    y1=sign(Z1);
else 
    y1=Z1;
end
v_2=-(beta_2/(1-K))*y1;

f_2=(2*x(5)+x(6))*tanh(x(5));
d_12=(a-2)*x(5)*tanh(x(5));

f_a2=3*x(5)*x(6);
G_a2=1/(1+x(6)^2);
d_22=x(5)*x(6)*(1+x(6)^2)*(b-3);

del_phi_2=-2-alpha*y1;

u_eq2=(1/G_a2)*(-f_a2+(del_phi_2)*f_2);
u_2=u_eq2+(1/G_a2)*v_2;

xdot(5)= f_2 + d_12;
xdot(6)= f_a2+G_a2*(u_2+d_22);
%%
z_3=x(8)+2*x(7)+alpha*abs(x(7));
rho_3=abs(x(7))*abs(x(8))+(2+alpha)*abs(x(7)); %x(1)^2; 
beta_3=rho_3+c;
Z2=z_3/epsilon2;
if abs(Z2)>1
    y2=sign(Z2);
else 
    y2=Z2;
end
v_3=-(beta_3/(1-K))*y2;

f_3=(2*x(7)+x(8))*tanh(x(7));
d_13=(a-2)*x(7)*tanh(x(7));

f_a3=3*x(7)*x(8);
G_a3=1/(1+x(8)^2);
d_23=x(7)*x(8)*(1+x(8)^2)*(b-3);

del_phi_3=-2-alpha*y2;

u_eq3=(1/G_a3)*(-f_a3+(del_phi_3)*f_3);
u_3=u_eq3+(1/G_a3)*v_3;

xdot(7)= f_3 + d_13;
xdot(8)= f_a3+G_a3*(u_3+d_23);
xdot=xdot';
end