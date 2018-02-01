%% #### -------------------------------------------------------------- #### 
%% George Kontoudis, Ph.D. Student, gpkont@vt.edu, www.georgekontoudis.com              
%% Virginia Tech, Department of Mechanical Engineering                         
%% ME 6544: Linear Control Theory     
%% Assignment 1 - Problem 1.4
%% Main - Transfer Function
%% #### -------------------------------------------------------------- #### 
close all;
clear all;
clc;

%% Magnitude and Phase
x=linspace(-10,10); % x=linspace(-10,0); % 3D Bode representation
y=linspace(-10,10);
[X,Y]=meshgrid(x,y);
Zp=20*log(abs((X+i.*Y+6)./((X+i.*Y+2.5-i.*2.5981).*(X+i.*Y+2.5+i.*2.5981))));
Za=angle((X+i.*Y+6)./((X+i.*Y+2.5-i.*2.5981).*(X+i.*Y+2.5+i.*2.5981)));
Zad=radtodeg(Za);

num=[1 6];
den=[1 5 13];

%% Figures

figure(1)
surfc(X, Y, Zp);
set(gca,'FontSize',16);
% set(gca,'XScale','log','YScale','log'); % 3D Bode representation
grid on;
xlabel('Real');ylabel('Imaginary');zlabel('Magnitude (dB)')

figure(2)
surfc(X, Y, Zad);
set(gca,'FontSize',16);
% set(gca,'XScale','log','YScale','log'); % 3D Bode representation
grid on
xlabel('Real');ylabel('Imaginary');zlabel('Angle (degrees)')

figure(3)
set(gca,'FontSize',16);
bode(num, den);
grid on;