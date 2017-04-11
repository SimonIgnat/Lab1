%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% KTH SD2231 - Applied Vehicle Control
%
%  Lab:     1 - Slip Control for rail and road vehicles
%  Date:    Spring term 2015
%  Teacher: Mohammad Mehdi Davari
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

global Veh
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Choose input parameters
Vehicle = 'Road'            % set Road or Rail for the vehicle parameters
mu_select = 2;              % set friction to mu_select = 1 (dry road), 2 (wet 
                            % road) or 3 (snow) for road and 1 for rail
Task = 2;
disturbance = 0;
dist_acc_time = [2 2.5];
dist_bra_time = [8.5 9];

dt = 0.001;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle parameters
switch Vehicle
    case {'Road'}
        % Initial values for road vehicle
        Veh.mc  = 395; % sprung mass of a quarter car in kg 
        Veh.mb = 5;
        Veh.mw = 25;%40; % unsprung mass of the quarter car in kg
        Veh.Jw = 1.7; % inertia of wheel in kgm^2
        Veh.kc = 300000; %bushing
        Veh.kb = 22000;%21000; % main spring coefficient in N/m
        Veh.kw = 2.4e5;%150000; % tire spring coefficient in N/m
        Veh.cc = 20;
        Veh.cb = 4500;%1500; % main damping coefficient in Ns/m
        Veh.cw = 0;%100; % tire damping coefficient in Ns/m
        Veh.Cs = 70000; % longitudinal tire stiffness in N/rad
        Veh.Cx = [1.6 1.65 1.1]; % Magic Formula tire shape factor
        Veh.Bx = [9 10 18]; % Magic Formula stiffness factor
        Veh.Dx= 9.81*(Veh.mc+Veh.mb+Veh.mw); % Magic Formula parameter
        Veh.Ex=0; % Magic Formula parameter
        Veh.k_V_low0=770*1; % low speed damping parameter in Ns/m
        Veh.V_low=2.5; % speed threshold in m/s
        Veh.sigma_min=0.02; % min relaxation length in m
        Veh.sigma_k0=0.1; % relaxation length in m
        Veh.Eps_F=0.01;  % Magic Formula division parameter
        Veh.fr = 0.008; % rolling resistance coefficient in -
        Veh.A = 2.2; % frontal cross-section area in m
        Veh.cw = 0.3; % aerodynamic drag coefficient in --
        Veh.r = 0.3; % rolling radius for tire in m
        Veh.mu = [1 0.7 0.3]; % friction coefficients [snow, rain and dry]
        tire_leg={'\mu = 1','\mu = 0.7','\mu = 0.3'};
        Veh.mu_dist = 0.3;        
        Veh.Cx_dist = 1.1; % Magic Formula tire shape factor
        Veh.Bx_dist = 18; % Magic Formula stiffness factor
        Veh.C_fk_dist=Veh.Bx_dist*Veh.Cx_dist*Veh.Dx;
        switch mu_select
            case {1}
                Veh.Bx = Veh.Bx(1); 
                Veh.Cx = Veh.Cx(1);
                Veh.mu = Veh.mu(1);
                run_time = 9;
                picture_name = 'pictures/road_dry_';
            case {2}
                Veh.Bx = Veh.Bx(2); 
                Veh.Cx = Veh.Cx(2);
                Veh.mu = Veh.mu(2);
                run_time = 12; 
                picture_name = 'pictures/road_wet_';               
            case {3}
                Veh.Bx = Veh.Bx(3); 
                Veh.Cx = Veh.Cx(3);
                Veh.mu = Veh.mu(3); 
                run_time = 25;  
                picture_name = 'pictures/road_snowy_';             
        end
        Veh.C_fk=Veh.Bx*Veh.Cx*Veh.Dx;

    case {'Rail'}
        % Initial values for rail vehicle
        Veh.mc  = 4375; % car body of a eighth car body in kg 
        Veh.mb = 1500;  % quarter mass of the bogie in kg
        Veh.mw = 3000; % unsprung mass of half a wheel set in kg
        Veh.Jw = 150; % inertia of wheel in kgm^2
        Veh.kc = 5*10^6; % secondary spring coefficient in N/m
        Veh.kb = 20*10^6; % primary spring coefficient in N/m
        Veh.kw = 2400e6; % wheel spring coefficient in N/m
        Veh.cc = 40*10^3; % secondary damping coefficient in Ns/m
        Veh.cb = 70*10^3; % primary damping coefficient in Ns/m
        Veh.cw = 0; % wheel damping coefficient in Ns/m
        Veh.Cs = 2400e6; % longitudinal tire stiffness in N/rad
        Veh.Cx = 1.0; % Magic Formula tire shape factor
        Veh.Bx = 50; % Magic Formula stiffness factor
        Veh.Dx= 9.81*(Veh.mc+Veh.mb+Veh.mw); % Magic Formula parameter
        Veh.Ex=0; % Magic Formula parameter
        Veh.k_V_low0=770*1; % low speed damping parameter in Ns/m
        Veh.V_low=2.5; % speed threshold in m/s
        Veh.sigma_min=0.02; % min relaxation length in m
        Veh.sigma_k0=0.2; % relaxation length in m
        Veh.Eps_F=0.01;  % Magic Formula division parameter
        Veh.C_fk=Veh.Bx*Veh.Cx*Veh.Dx;
        Veh.fr = 0.002; % rolling resistance coefficient in -
        Veh.A = 10; % frontal cross-section area in m
        Veh.cw = 0.26; % aerodynamic drag coefficient in --
        Veh.r = 0.46; % rolling radius for tire in m
        Veh.mu = 0.2; % friction coefficients [snow, rain and dry]  
        tire_leg={'\mu = 0.2'};
        mu_select = 1;
        Veh.mu_dist = Veh.mu;        
        Veh.Cx_dist = Veh.Cx; % Magic Formula tire shape factor
        Veh.Bx_dist = Veh.Bx; % Magic Formula stiffness factor
        Veh.C_fk_dist=Veh.Bx_dist*Veh.Cx_dist*Veh.Dx;
        run_time = 30;
        picture_name = 'pictures/rail_';
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Driver parameters

%Included by Eric
%The value to which the program will accelerate is not 90 km/h. TO change
%it just change the value in slip_model_Student/Driver/Switch/Threshold
%(default Dri.v_lim/3.6). Attention there are two different Switches
%(Switch and Switch1)
%For Task 2.b probably use Dri.v_lim as threshold as I understand

Dri.v_lim = 90; %in km/h
Dri.t_const = 3; %in seconds
Dri.Ka = 1;  % throttle gain (0-100%) of driver
Dri.Kb = -1; % braking gain (0-100%) of driver

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control parameters
T_max = (Veh.mc+Veh.mb+Veh.mw)*9.81*Veh.r;

%Included by Eric
%the next two variables (K_em, K_brake) convert signal (1 for acc, 0 for nothing, -1 for
%breaking) into the applied torque to the wheels. I included them into
%slip_model_Student/Controller/FeedForward
%for different cases (variable 'Vehicle') and for different frictions
%(variable 'mu_select') the wheel can have high slip. To see this just open
%the Scope 'vx' in the main view (slip_model_student)

%Changing the values for K_em and K_brake (default 0.4 and 0.5) will lead
%to higher applied torque on the wheels and can cause higher acceleration
%and shorter time to reach max velocity (variable 'Dri.v_lim')
%Good example could be :
%mu_select=1 , K_em = 1*T_max in comparisson with K_em = 1.2*T_max





K_em = 0.735*T_max;%0.4*T_max;
K_brake = 0.725*T_max;

%Included by Eric
% P Controller variables
% Road vehicle


controller = 'Optimal'

alpha_brake = 100;
alpha_em = 100;

switch(controller)
    case 'P'
%         switch(Vehicle)
%             case 'Road'
                Kp_em = 100;
                Ki_em = 0;
                Kd_em = 0;

                Kp_brake = 10;
                Ki_brake = 0;
                Kd_brake = 0;
%             case 'Rail'
%                 Kp_em = 1;
%                 Ki_em = 0;
%                 Kd_em = 0;
% 
%                 Kp_brake = 1;
%                 Ki_brake = 0;
%                 Kd_brake = 0;
%         end
    case 'PI'
%         switch(Vehicle)
%             case 'Road'
                Kp_em = 50;
                Ki_em = 1;
                Kd_em = 0;

                Kp_brake = 50;
                Ki_brake = 0.01;
                Kd_brake = 0;
%             case 'Rail'
%                 Kp_em = 1; % Seems to be close to optimum
%                 Ki_em = 2;
%                 Kd_em = 0;
% 
%                 Kp_brake = 4;
%                 Ki_brake = 1;
%                 Kd_brake = 0;
%         end
     case 'PD'
%         switch(Vehicle)
%             case 'Road'
                Kp_em = 100;
                Ki_em = 0;
                Kd_em = 2;

                Kp_brake = 100;
                Ki_brake = 0;
                Kd_brake = 2;
%             case 'Rail'
%                 Kp_em = 1; % Seems to be close to optimum
%                 Ki_em = 2;
%                 Kd_em = 0;
% 
%                 Kp_brake = 4;
%                 Ki_brake = 1;
%                 Kd_brake = 0;
%         end   
    case 'PID'
%         switch(Vehicle)
%             case 'Road' % Optimal so far
                Kp_em = 50; 
                Ki_em = 2;
                Kd_em = 2;

                Kp_brake = 100;
                Ki_brake = 1;
                Kd_brake = 4;
%             case 'Rail'
%                 Kp_em = 1;
%                 Ki_em = 2;
%                 Kd_em = 0.5;
% 
%                 Kp_brake = 4; % Seems to be close to optimum
%                 Ki_brake = 1;
%                 Kd_brake = 10;
%         end
    case 'Optimal'
        switch(Vehicle)
            case 'Road'   
                Kp_em = 50; 
                Ki_em = 2;
                Kd_em = 2;

                Kp_brake = 100;
                Ki_brake = 0;
                Kd_brake = 2;

            case 'Rail'
                Kp_em = 30; 
                Ki_em = 4;
                Kd_em = 2;      

                Kp_brake = 300;
                Ki_brake = 1;
                Kd_brake = 7;
        end
        
        
    case 'FeedThrough'
        switch(Vehicle)
            case 'Road'   
                Kp_em = 0; 
                Ki_em = 0;
                Kd_em = 0;      
 
                Kp_brake = 0;
                Ki_brake = 0;
                Kd_brake = 0;
                
                switch(mu_select)
                    case 1
                        K_em = 1.1*1.05*T_max;
                        K_brake = 1.1*1.03*T_max;
                        
                    case 2
                        K_em = 0.735*1.0*T_max;%0.4*T_max;
                        K_brake = 0.725*1.0*T_max;
                    case 3
                        K_em = 0.322*1.0*T_max;
                        K_brake = 0.31*1.0*T_max;
                end
                
                
            case 'Rail'
                Kp_em = 0; 
                Ki_em = 0;
                Kd_em = 0;      

                Kp_brake = 0;
                Ki_brake = 0;
                Kd_brake = 0;
                
                
                
                K_em = 0.22*1.0*T_max;
                K_brake = 0.23*1.0*T_max;
        end
end





%% tire model - force-slipcurves
slip0 = -1:0.01:1;
mu_plot = diag(Veh.mu)*sin(atan(slip0'*Veh.Bx)*diag(Veh.Cx))';
% Fz_max = [mu';-mu'].*(m+mt)*9.81;grid on, 
if 0
hold on
plot(slip0,mu_plot,'LineWidth',2),grid on, hold on
axis([-1 1 -1.1 1.1])
% plot([a(1) a(end)],[1;1]*Fz_max(1),[a(1) a(end)],[1;1]*Fz_max(2),[a(1) a(end)],[1;1]*Fz_max(3),[a(1) a(end)],[1;1]*Fz_max(4),[a(1) a(end)],[1;1]*Fz_max(5),[a(1) a(end)],[1;1]*Fz_max(6),'LineStyle','--','LineWidth',2,'Color',[0 0 0])
xlabel('longitudinal slip \kappa/rad')
ylabel('longitudinal force f_x/N')
legend(tire_leg(mu_select),'Location','NorthWest')
end







[~,b] = max(mu_plot);
slip_ref_t = slip0(b);
[~,b] = min(mu_plot);

slip_ref_b = slip0(b);
if ~strcmp(controller,'FeedThrough')
    slip_ref_b = -0.14;
    
    slip_ref_t = 0.14;
end

%% 
% definition of state-space model for three mass quarter car model
% z(1)=! xc
% z(2)=! xcd = zd(1)
% z(3)=! xb  
% z(4)=! xbd = zd(3)
% z(5)=! xw  
% z(6)=! xwd = zd(5)
% this gives:
% zd(1)= z(2)   = xcd
% zd(2)= zdd(1) = xcdd
% zd(3)= z(4)   = xbd
% zd(4)= zdd(3) = xbdd
% zd(5)= z(6)   = xwd
% zd(6)= zdd(5) = xwdd

AA = [0 1 0 0 0 0; -Veh.kc/Veh.mc -Veh.cc/Veh.mc Veh.kc/Veh.mc Veh.cc/Veh.mc 0 0; 0 0 0 1 0 0; Veh.kc/Veh.mb Veh.cc/Veh.mb -(Veh.kc+Veh.kb)/Veh.mb -(Veh.cc+Veh.cb)/Veh.mb Veh.kb/Veh.mb Veh.cb/Veh.mb; 0 0 0 0 1 0; 0 0 Veh.kb/Veh.mw Veh.cb/Veh.mw -(Veh.kb+Veh.kw)/Veh.mw -(Veh.cb+Veh.cw)/Veh.mw];
BB = [0 0; 0 0; 0 0; 0 0; 0 0; Veh.kw/Veh.mw Veh.cw/Veh.mw];
% CC = eye(6); % use for motions
% DD = [0 0;0 0;0 0;0 0;0 0;0 0;]; % use for motions
CC = [0 0 0 0 -Veh.kw -Veh.cw]; % use for dynamic load
DD = [Veh.kw Veh.cw]; % use for dynamic load

%modified by Eric
%to automatically run the simulink file - euals pressing the green play
%button in simulink
sim('slip_model_Student')
if 0
    figure(1)
    plot(slip_data.time,slip_data.signals.values(:,1),'r','LineWidth',2);
    hold on;
    plot(slip_data.time,slip_data.signals.values(:,2),'b','LineWidth',2);
    plot([-1 1e4],[slip_ref_t slip_ref_t],'LineStyle','- -','Color','k');
    grid on;
    title('Slip for acceleration and brake phase');
    ylabel('Longitudinal Slip')
    xlabel('Time in seconds')
    ylim([0 1.1]);
    legend('Slip tractive','Slip brake');
    %Set a good xlim !!!
    xlim([0 run_time]);
end 

if 0
    figure(2)
    plot(feedtrough_vx.time,feedtrough_vx.signals.values(:,1),'r','LineWidth',2);
    hold on;
    plot(feedtrough_vx.time,feedtrough_vx.signals.values(:,2),'b','LineWidth',2);
    grid on;
    
    title('Acceleration and braking with feed-through');
    ylabel('Longitudinal velocity')
    xlabel('Time in seconds')
    %ylim([0 1.1]);
    legend('V_x','\omega');
    %Set a good xlim !!!
    xlim([0 run_time]);
end;

if 1
    figure(3)
    plot(utilized_fri.time,utilized_fri.signals.values(:),'r','LineWidth',2);
    grid on;
    hold on;
    plot([0 run_time],[ max(mu_plot)  max(mu_plot)],'LineStyle','- -','Color','k');
    plot([0 run_time],[ min(mu_plot)  min(mu_plot)],'LineStyle','- -','Color','k');
    title('Utilized friciton of the vehicle');
    ylabel('Utilized friciton in percent');
    xlabel('Time in seconds')
    ylim([-1.1 1.1]);
    %Set a good xlim !!!
    xlim([0 run_time]);
end;

[~,b_acc_time] = min(Out_Throttle);
acc_time = feedtrough_vx.time(b_acc_time)
[~,b_break_time] = min(Out_Brakepedal);
bra_dist = max(Out_sx) - Out_sx(b_break_time)
