%% Introduction to PID-controller design
%  This introduction gives you a short overview how the different parts of
%  a PID-controller work.

%%%%%%%%%%%%%%%%%%%%%%%% NOTE %%%%%%%%%%%%%%%%%%%%%%%%%%
% Do not forget to add ypur system's transfer function before running the
% code!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clc
clear all

%% mass, spring and damper values and transfer function
%Modified by Eric
M =20;% 395;%values taken from inital.m file - 20 was predefined                    % mass in kg - 395 Kg
K = 400;%22000;%values taken from inital.m file - 400 was predefined                   % spring stiffness coeficient in N/m - 22000
C = 200;%1500;%values taken from inital.m file - 200 was predefined                    % damping coefficient in Ns/m    4500
s = tf('s');                % LaPlace parameter




t = 0:0.01:2;
%% plot the step response of the different controllers (P,PD,PI and PID)
KP = 5000;
KI = 0;
KD = 0;

subplot(2,2,1)
T =  1/(M*s*s+C*s+K);
Ctrl = pid(5000,0,0);
S = feedback(Ctrl*T,1);
step(S);
grid on;
title('Step response - P-Controller');
xlim([0 2]);
ylim([0 1.4]);
subplot(2,2,2)
T =  1/(M*s*s+C*s+K);
Ctrl = pid(5000,0,100);
S = feedback(Ctrl*T,1);
step(S);
grid on;
title('Step response - PD-Controller');
xlim([0 2]);
ylim([0 1.4]);
subplot(2,2,3)
T =  1/(M*s*s+C*s+K);
Ctrl = pid(600,1400,0);
S = feedback(Ctrl*T,1);
step(S);
grid on;
title('Step response - PI-Controller');
xlim([0 2]);
ylim([0 1.4]);
subplot(2,2,4)
T =  1/(M*s*s+C*s+K);
Ctrl = pid(5000,5000,600);
S = feedback(Ctrl*T,1);
step(S);
grid on;
title('Step response - PID-Controller');
xlim([0 2]);
ylim([0 1.4]);


%% Included by Eric
%Task 1.d
%pzmap(T) %two stable poles, system is stable - P = 0 (number of unstable
%poles as defined in the lecture
%nyquist(pid(6e3,0e5,0e5)*T) %P-Controller Z = 0 (number of encircled -1) , N = 0 --> stable
%nyquist(pid(2e5,0e5,1e4)*T) %PD-Controller Z = 0, N = 0 --> stable
%nyquist(pid(1e1,2e4,0e4)*T) %PI-Controller Z = 0, N = 0 --> stable
%nyquist(pid(2e5,5e5,5e5)*T) %PID-Controller Z = 0, N = 0 --> stable