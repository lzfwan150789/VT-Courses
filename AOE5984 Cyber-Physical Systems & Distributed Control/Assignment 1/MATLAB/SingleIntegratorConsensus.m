%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% AOE 5984: Cyber-Physical Systems & Distributed Control      
%% Assignment 1, Single-Integrator Consensus for Different Graph Topologies 
%% ODE sln's
%% #### -------------------------------------------------------------- #### 
clear all;
close all;
clc;

tic; % start time estimation

x0 = [1 -1 3 -3 5 -5]'; % initial values
x0_1 = [4.5 5 5.2 3 2 0 0 0 0 0 0 0 0 0 0 0 0 0]'; % initial values
x0_2 = [45 -25 -30 -75 -85 -120 0 0 0 0 0 0 0 0 0 0 0 0]'; % initial values
t_int1 = [0 10]; % spanning time
t_int2 = [0 8]; % spanning time
t_int3 = [0 2]; % spanning time
t_int4 = [0 15]; % spanning time
t_int5 = [0 40]; % spanning time


[t,x] = ode23('dirGraph', t_int1, x0); % solve odes
[t_un,x_un] = ode23('undirGraph', t_int2, x0); % solve odes
[t_co,x_co] = ode23('compGraph', t_int3, x0); % solve odes
[t_dt,x_dt] = ode23('ditrGraph', t_int1, x0); % solve odes
[t_us,x_us] = ode23('unstarGraph', t_int1, x0); % solve odes
[t_ds,x_ds] = ode23('distarGraph', t_int1, x0); % solve odes
[t_u6c,x_u6c] = ode23('u6cGraph', t_int1, x0); % solve odes
[t_d6c,x_d6c] = ode23('d6cGraph', t_int4, x0); % solve odes
[t_u5,x_u5] = ode23('u5Graph', t_int4, x0); % solve odes
[t_3s,xp_3s] = ode23('dir3sGraph', t_int4, x0_1); % solve odes
[t_dt3s,xp_dt3s] = ode23('dt3sGraph', t_int4, x0_2); % solve odes



figure (1)
subplot(1,3,1);
plot(t,x);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Direct graph');

subplot(1,3,2);
plot(t_un,x_un);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Undirect graph');

subplot(1,3,3);
plot(t_co,x_co);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Complete graph K^6');

figure (2)
subplot(1,3,1);
plot(t_dt,x_dt);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Direct tree graph');

subplot(1,3,2);
plot(t_us,x_us);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Undirect star K_{1.5}');

subplot(1,3,3);
plot(t_ds,x_ds);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Direct star');



figure (3)
subplot(1,3,1);
plot(t_u6c,x_u6c);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Undirect 6-cycle graph');

subplot(1,3,2);
plot(t_d6c,x_d6c);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Directed 6-cycle');

subplot(1,3,3);
plot(t_u5,x_u5);
xlabel('Time [s]');
ylabel('States');
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Undirected path P^5');



figure (4)
subplot(2,2,1);
plot(t_3s,xp_3s(:,1:6));
xlabel('Time [s]');
ylabel('Heading angle');
%legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Convergence of headings - Direct graph');

subplot(2,2,2);
plot(xp_3s(:,7:12), xp_3s(:,13:18));
xlabel('State x');
ylabel('State y');
%legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Nodes converge to consensous heading - Direct graph');

subplot(2,2,3);
plot(t_dt3s,xp_dt3s(:,1:6));
xlabel('Time [s]');
ylabel('Heading angle');
%legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Convergence of headings - Direct tree graph');

subplot(2,2,4);
plot(xp_dt3s(:,7:12), xp_dt3s(:,13:18));
xlabel('State x');
ylabel('State y');
%legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
title('Nodes converge to consensous heading - Direct tree graph');

toc; % finish time estimation
