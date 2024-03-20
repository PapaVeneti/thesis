%% Algorithm Report
alg_fig = figure(name='Algorithm');
set(alg_fig,'defaultAxesColorOrder',[ 0,0,0 ; 0,0,1]);
hold on
yyaxis left
plot(out.sol_status_leg_mpc,'k',Linewidth=2)
set(gca,"Ytick",[0,1,2,3,4], "YTickLabel",{'SUCCESS','NAN DETECTED','MAXITER','MINSTEP','QP FAILURE'},"FontSize",15)
ylim([0,4])

yyaxis right
plot(out.reset_state,'b',Linewidth=2)
set(gca,"Ytick",[0,1], "YTickLabel",{'TORQUE','RESET'},"FontSize",15)

grid on
ylim([0,3])

title('$Torque-Reset \  algorithm$',interpreter ='latex',fontsize=25)
legend('$mpc\ solution\ status$','$reset \ state$',interpreter ='latex',fontsize=20)

%% sol  time

mean_sol_time   = mean(out.solution_time.Data,'omitnan');
std_sol_time    = std (out.solution_time.Data,'omitnan');
max_sol_time    = max (out.solution_time.Data);
disp(['Mean solution time is: ', num2str( 1e3*mean_sol_time,3 ),'ms']);
disp(['Standard deviation is: ', num2str( 1e3*std_sol_time ,3 ),'ms']);
disp(['Max solution time is: ' , num2str( 1e3*max_sol_time ,3 ),'ms']);

%% Produced torque read
time_tau_sim = out.t_fr_comp.Time;
time_q_sim   = out.q_fr.Time;

%fr : 
q_MH_fr_raw  = out.q_fr.Data(:,1);
q_MH_fr      = interp1(time_q_sim,q_MH_fr_raw,time_tau_sim); %interpolate
u_fr_mpc     = out.t_fr_comp.Data;
u_fr_pid     = out.t_fr_servo.Data;

tau_mpc_fr   = base_torque_calc(q_MH_fr,u_fr_mpc);
tau_total_fr = base_torque_calc(q_MH_fr,u_fr_mpc+u_fr_pid);

tau_mpc_fr   ( isnan( tau_mpc_fr  )) = 0;
tau_total_fr ( isnan( tau_total_fr)) = 0;

%saturate_total_torque_fr
for i=1:3
    l_id = tau_total_fr(:,i) <leg_model_obj.input_constraints(i,1);
    u_id = tau_total_fr(:,i) >leg_model_obj.input_constraints(i,2);

    tau_total_fr(l_id,i) = leg_model_obj.input_constraints(i,1);
    tau_total_fr(u_id,i) = leg_model_obj.input_constraints(i,2);
end

%fl : 
% q_MH_fl_raw  = out.q_fl.Data(:,1);
% q_MH_fl      = interp1(time_q_sim,q_MH_fl_raw,time_tau_sim); %interpolate
% u_fl_mpc     = out.t_fl_comp.Data;
% u_fl_pid     = out.t_fl_servo.Data;
% 
% tau_mpc_fl   = base_torque_calc(q_MH_fl,u_fl_mpc);
% tau_total_fl = base_torque_calc(q_MH_fl,u_fl_mpc+u_fl_pid);
% 
%
%
% %rr : 
% q_MH_rr_raw  = out.q_rr.Data(:,1);
% q_MH_rr      = interp1(time_q_sim,q_MH_rr_raw,time_tau_sim); %interpolate
% u_rr_mpc     = out.t_rr_comp.Data;
% u_rr_pid     = out.t_rr_servo.Data;
% 
% tau_mpc_rr   = base_torque_calc(q_MH_rr,u_rr_mpc);
% tau_total_rr = base_torque_calc(q_MH_rr,u_rr_mpc+u_rr_pid);
% 
% %rl : 
% q_MH_rl_raw  = out.q_rl.Data(:,1);
% q_MH_rl      = interp1(time_q_sim,q_MH_rl_raw,time_tau_sim); %interpolate
% u_rl_mpc     = out.t_rl_comp.Data;
% u_rl_pid     = out.t_rl_servo.Data;
% 
% tau_mpc_rl   = base_torque_calc(q_MH_rl,u_rl_mpc);
% tau_total_rl = base_torque_calc(q_MH_rl,u_rl_mpc+u_rl_pid);

%% Torque plots: [t_MH]
figure(Name= 'Comparison: [u]')
mh_index = {'X','Y','Z'};

title_str   = '$Produced\ torque] $';
ylabel_names = naming_automation('\tau',mh_index,'$','$');
% ref_names    = naming_automation('\tau',mh_index,'$reference:\ ','$');
mpc_names   = naming_automation('\tau',mh_index,'$MPC\ optimized:','$');
pid_names   = naming_automation('\tau',mh_index,'$Compensated:','$');


initialize_plot(3,"title",title_str ,"ylab",ylabel_names)
% populate_plot(3,tmpc(1:end-1),ones(N,1)*tref_plot',linestyle='--',color='g',display_names=ref_names)
populate_plot(3,time_tau_sim,tau_mpc_fr,"display_names",mpc_names,color='k',linewidth=1.75)
populate_plot(3,time_tau_sim,tau_total_fr,"display_names",pid_names,linestyle='--',color='b')
ylim([-15,15])

%% torque in each cycle
Tsim    = max(time_tau_sim);
Thl     = 0.1;
Ncycles = floor(Tsim/Thl);
n_cycle = floor( (length(time_tau_sim)-1)/Ncycles );

tcycle       = zeros(Ncycles,3);
tcycle_total = zeros(Ncycles,3);

for i = 1:Ncycles
    index = (1+(i-1)*n_cycle) :i*n_cycle; 
    tcycle(i,:)       = trapz(time_tau_sim( index ) , tau_mpc_fr(index,:));
    tcycle_total(i,:) = trapz(time_tau_sim( index ) , tau_total_fr(index,:));
end



figure(name=  'net_torque')
title_str   = '$Produced\ torque\ in\ each\ cycle:$';
initialize_plot(3,"title",title_str ,"ylab",ylabel_names,"subplot_titles",{'1','2','3'})
populate_plot(3,linspace(0,Tsim,Ncycles+1),[tcycle      ;0,0,0],"display_names",mpc_names,color='k',linewidth=1.75,stairs=true)
populate_plot(3,linspace(0,Tsim,Ncycles+1),[tcycle_total;0,0,0],"display_names",pid_names,color='b',linewidth=1.75,stairs=true)
% populate_plot(3,time_tau_sim,Tintegral_cum,"display_names",mpc_names,color='k',linewidth=1.75,stairs=true)


% 
Tintegral      = trapz(time_tau_sim,tau_mpc_fr);
Tintegral_tot  = trapz(time_tau_sim,tau_total_fr);

mpc_index_torques = {'X,MPC','Y,MPC','Z,MPC'};
tot_index_torques = {'X,TOT','Y,TOT','Z,TOT'};

torques_txt_mpc = naming_automation('\int{ \tau',mpc_index_torques,'$','dt} =');
torques_txt_tot = naming_automation('\int{ \tau',tot_index_torques,' ','dt} =');
torques_txt = {'','',''};
for i =1:3
    torques_txt_mpc{i} = [torques_txt_mpc{i}, num2str(Tintegral(i))    ];
    torques_txt_tot{i} = [torques_txt_tot{i}, num2str(Tintegral_tot(i)),'\ \ [Nm \ s] $'];
    torques_txt{i}     = [torques_txt_mpc{i},' \ || \',torques_txt_tot{i}  ];
end

for i =1:3
subplot(3,1,i)
title(torques_txt{i},Interpreter='latex',FontSize=25)
end 

