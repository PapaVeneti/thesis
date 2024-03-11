
XX = extract_traj([0,1.5],out.x_traj.Data(1,:),10);
UU = extract_traj([0,1.5],out.u_traj.Data(1,:),3);


tsimulink = out.q.Time;
q_sim     = out.q.Data;
u_sim_MH    = out.tMH.Data;
u_sim_11    = out.t11.Data;
u_sim_12    = out.t12.Data;
ntau        = size(u_sim_MH,3);
u_sim       = zeros(ntau,3);
u_sim(:,1)  = reshape(u_sim_MH,[ntau,1]);
u_sim(:,2)  = reshape(u_sim_11,[ntau,1]);
u_sim(:,3)  = reshape(u_sim_12,[ntau,1]);

% u_sim = u_sim_
%% Comparison plots: [q]
indices = {'MH','11','21','12','22'};
tmpc    = linspace(0,Th,N+1)';
xref_plot = xref;
xref_plot([3,5]) = nan;

ref_names =  {'$q_{r,MH}$','$q_{r,11}$','','$q_{r,12}$',''};
% q_sim = out.q.Data;
% tsimulink = out.q.Time;
% w_sim = out.w.Data;

figure(Name= 'Comparison: [q]')
title_str   = '$Acados \ optimized\ trajectory:\ [q] $';
ylabel_names = naming_automation('q',indices,'$','$');
sim_names    = naming_automation('q',indices,'$','$');
simu_names    = naming_automation('q',indices,'$simulink:\ ','$');

initialize_plot(5,"title",title_str ,"ylab",ylabel_names )
% populate_plot(5,[],xref_plot(1:5)',"reference",true,linestyle='--',color='g',display_names=ref_names)
populate_plot(5,tmpc,XX(:,2:6),"display_names",sim_names,linestyle='--',color='b')
populate_plot(5,tsimulink,q_sim,"color",'k',linestyle='-',display_names=simu_names)

%% Comparison plots: [u]
figure(Name= 'Comparison: [u]')

title_str   = '$Acados \ optimized\ trajectory:\ [\tau] $';
ylabel_names = naming_automation('\tau',indices([1,2,4]),'$','$');
sim_names    = naming_automation('\tau',indices([1,2,4]),'$sim:\ ','$');
simu_names   = naming_automation('\tau',indices([1,2,4]),'$simulink:\ ','$');

initialize_plot(3,"title",title_str ,"ylab",ylabel_names )
populate_plot(3,UU(:,1),UU(:,2:4),"display_names",sim_names)
populate_plot(3,tsimulink,u_sim,"display_names",sim_names,"color",'b',linestyle='--')
