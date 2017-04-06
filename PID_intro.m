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

123
T =  1/(M*s*s+C*s+K);
Ctrl = pid(KP,KI,KD);
S = feedback(Ctrl*T,1);
figure
step(S);
figure(3)
set(gcf,'Position',[100 100 600 600])
grid on
daten = line(0,0,'LineWidth',3);
title('Step response of spring-damper-mass system (START: no controller active) ','FontSize',12)
xlabel('Time (sec)','FontSize',12)
ylabel('Amplitude')
 xlim([-0 max(t)])
ylim([-0.5 1.5])
% Slider erzeugen --> the guy who wrote that seems to be german ;)

%Modified by Eric (Sliders maximum values are increased
%original Pmax = 10000, Imax = 10000, Dmax = 2000)
KP_Slider = uicontrol('Style', 'slider',...
'Position', [100 75 120 20],'min',0,'max',200000,...
'value',KP,'String','Kp');
text(0.2,-0.33,'K_{p}=','FontSize',14)
KP_wert=text(0.4,-0.33,'test','FontSize',14);

KI_Slider = uicontrol('Style', 'slider',...
'Position', [250 75 120 20],'min',0,'max',500000,...
'value',KI,'String','Ki');
text(0.8,-0.33,'K_{i}=','FontSize',14)
KI_wert=text(1.0,-0.33,num2str(KI),'FontSize',14);

KD_Slider = uicontrol('Style', 'slider',...
'Position', [400 75 120 20],'min',0,'max',1000000,...
'value',KD,'String','Kd');
text(1.5,-0.33,'K_{d}=','FontSize',14)
KD_wert=text(1.7,-0.33,num2str(KD),'FontSize',14);

M_Slider = uicontrol('Style', 'slider',...
'Position', [100 125 120 20],'min',1,'max',400,...
'value',M,'String','M');
text(0.2,-0.1,'M=','FontSize',14)
M_wert=text(0.4,-.1,num2str(M),'FontSize',14);

K_Slider = uicontrol('Style', 'slider',...
'Position', [250 125 120 20],'min',0,'max',25000,...
'value',K,'String','K');
text(0.8,-0.1,'K=','FontSize',14)
K_wert=text(1.0,-0.1,num2str(K),'FontSize',14);

C_Slider = uicontrol('Style', 'slider',...
'Position', [400 125 120 20],'min',0,'max',2000,...
'value',C,'String','C');
text(1.5,-0.1,'C=','FontSize',14)
C_wert=text(1.7,-0.1,num2str(C),'FontSize',14);

while(1)
KP = get(KP_Slider,'value');
KI = get(KI_Slider,'value');
KD = get(KD_Slider,'value');
M = get(M_Slider,'value');
K = get(K_Slider,'value');
C = get(C_Slider,'value');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implement the transfer function of the equation of motion here:
% Use variable s for the first derivative of x and s^2 for second
% derivative of x
T =  1/(M*s*s+C*s+K); %Included by Eric
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ctrl = pid(KP,KI,KD);
S = feedback(Ctrl*T,1);
set(daten,'xdata',t,'ydata',step(S,t));
set(KP_wert,'string',num2str(KP))
set(KI_wert,'string',num2str(KI))
set(KD_wert,'string',num2str(KD))
set(M_wert,'string',num2str(M))
set(K_wert,'string',num2str(K))
set(C_wert,'string',num2str(C))
drawnow
end


%% Included by Eric
%Task 1.d
%pzmap(T) %two stable poles, system is stable - P = 0 (number of unstable
%poles as defined in the lecture
%nyquist(pid(6e3,0e5,0e5)*T) %P-Controller Z = 0 (number of encircled -1) , N = 0 --> stable
%nyquist(pid(2e5,0e5,1e4)*T) %PD-Controller Z = 0, N = 0 --> stable
%nyquist(pid(1e1,2e4,0e4)*T) %PI-Controller Z = 0, N = 0 --> stable
%nyquist(pid(2e5,5e5,5e5)*T) %PID-Controller Z = 0, N = 0 --> stable