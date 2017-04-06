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
M =395;%values taken from inital.m file - 20 was predefined                    % mass in kg - 395 Kg
K = 22000;%values taken from inital.m file - 400 was predefined                   % spring stiffness coeficient in N/m - 22000
C = 4500;%values taken from inital.m file - 200 was predefined                    % damping coefficient in Ns/m    4500
s = tf('s');                % LaPlace parameter




t = 0:0.01:2;
T =  1/(M*s*s+C*s+K);
%% plot the step response of the different controllers (P,PD,PI and PID)
if 1
    subplot(2,2,1)
    Ctrl = pid(2e5,0,0);%pid(5000,0,0);
    S = feedback(Ctrl*T,1);
    step(S);
    grid on;
    title('Step response - P-Controller');
    xlim([0 2]);
    ylim([0 1.4]);
    subplot(2,2,2)
    T =  1/(M*s*s+C*s+K);
    Ctrl = pid(2e5,0,5e3);%pid(5000,0,100);
    S = feedback(Ctrl*T,1);
    step(S);
    grid on;
    title('Step response - PD-Controller');
    xlim([0 2]);
    ylim([0 1.4]);
    subplot(2,2,3)
    T =  1/(M*s*s+C*s+K);
    Ctrl = pid(8e3,7.5e4,0);%pid(600,1400,0);
    S = feedback(Ctrl*T,1);
    step(S);
    grid on;
    title('Step response - PI-Controller');
    xlim([0 2]);
    ylim([0 1.4]);
    subplot(2,2,4)
    T =  1/(M*s*s+C*s+K);
    Ctrl = pid(1.7e5,3e5,2e4);%pid(5000,5000,600);
    S = feedback(Ctrl*T,1);
    step(S);
    grid on;
    title('Step response - PID-Controller');
    xlim([0 2]);
    ylim([0 1.4]);
end

%% Included by Eric
%Task 1.d
%pzmap(T) %two stable poles, system is stable - P = 0 (number of unstable
%poles as defined in the lecture
%nyquist(pid(2e5,0,0)*T) %P-Controller Z = 0 (number of encircled -1) , N = 0 --> stable
%nyquist(pid(2e5,0,5e3)*T) %PD-Controller Z = 0, N = 0 --> stable
%nyquist(pid(8e3,7.5e4,0)*T) %PI-Controller Z = 0, N = 0 --> stable
%nyquist(pid(1.7e5,3e5,2e4)*T) %PID-Controller Z = 0, N = 0 --> stable