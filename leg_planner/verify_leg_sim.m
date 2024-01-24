% This script verifies that the simulink simulation is in accordance with
% the matlab simulation and `ntnu_leg_ode.m` modelling or `ntnu_leg_closed.m`

clc;
clear all;
close all;
%% Simulation parameters (time, input, initial conditions
sim_time = [0 10];
controller_rate = 1e-3;
qinitial   = [0;0;0;0;0];
qinitial_t = [0;0;0;0;0];
KD_definition


% --- : random input signal
% ufunc = @(t,y) [1;1;1]*0.005 + [1;0;0] + 0*t;
% ufunc = @(t,y) [2;1;1]*0.1*sin(10*t);
% ufunc = @(t,y) [2;1;1]*0.05*sin(10*t);
%% Simulation (ode)
%1.  Open chain
% opts = odeset(AbsTol=1e-6,RelTol=1e-3);
% controlled_leg = @(t,y) ntnu_leg_ode([],y,[0;0;0;0;0] );
% [t,y] = ode45(@(t,y) controlled_leg(t,y),sim_time,[qinitial;qinitial_t],opts);

%1.  Closed chain
% opts = odeset(AbsTol=1e-9,RelTol=1e-6,InitialStep=1e-4,MaxStep=1e-5);
opts = odeset(AbsTol=1e-6,RelTol=1e-3);
controlled_leg = @(t,y) ntnu_leg_closed([],y,[0;0;0],D );
[t,y] = ode45(@(t,y) controlled_leg(t,y),sim_time,[qinitial;qinitial_t],opts);


qin = [t,y(:,1:5 )];
win = [t,y(:,6:10)];

%% simulink sim
% `From workspace`: 
%   - No interpolation
%   - Holding final value
% time = sim_time(1):controller_rate:sim_time(end);
% tau = ufunc(time,[]);
% tau = [time' tau'] ;

% out = sim('simulink/ntnu_leg_motion_generation.slx',sim_time);
 out = sim('simulink/ntnu_leg.slx',sim_time);


%% Comparison plots: [q]
indices = {'MH','11','21','12','22'};
tq    = out.q.Time;
q_sim = out.q.Data;
w_sim = out.w.Data;

figure(Name= 'Comparison: [q]')
title_str   = '$Comparison\ between\ custom\ simulation\ and\ simulink:\ [q] $';
ylabel_names = naming_automation('q',indices,'$','$');
sim_names    = naming_automation('q',indices,'$sim:\ ','$');
simu_names   = naming_automation('q',indices,'$simulink:\ ','$');

initialize_plot(5,"title",title_str ,"ylab",ylabel_names )
populate_plot(5,t,y(:,[1 2 3 4 5]),"display_names",sim_names)
populate_plot(5,tq,q_sim,"color",'b',linestyle='--',display_names=simu_names)

%% Comparison plots: [w]
figure(Name= 'Comparison: [w]')

title_str   = '$Comparison\ between\ custom\ simulation\ and\ simulink:\ [\omega] $';
ylabel_names = naming_automation('\omega',indices,'$','$');
sim_names    = naming_automation('\omega',indices,'$sim:\ ','$');
simu_names   = naming_automation('\omega',indices,'$simulink:\ ','$');


initialize_plot(5,"title",title_str ,"ylab",ylabel_names )
populate_plot(5,t,y(:,[6 7 8 9 10]),"display_names",sim_names)
populate_plot(5,tq,w_sim,"color",'b',linestyle='--',display_names=simu_names)

