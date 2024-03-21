Th = 0.75;
N=150;
model = leg_model;

a =load('Trajectories/N150Th075_rl_01.mat');
xtraj1 = a.xtraj;
xtraj1 = circshift(xtraj1,ceil(N/2),2);

b =load('Trajectories/N150Th075_lr_02.mat');
xtraj2 = b.xtraj;



%% 8.b. Comparison plots: [q]
indices = {'MH','11','21','12','22'};
tmpc    = linspace(0,Th,N+1)';
ref_names =  {'$q_{r,MH}$','$q_{r,11}$','','$q_{r,12}$',''};


figure(Name= 'Comparison: [q]')
title_str   = '$Acados \ optimized\ trajectory:\ [q] $';
ylabel_names = naming_automation('q',indices,'$','$');
sim1_names    = naming_automation('q',indices,'$1st:\ ','$');
sim2_names    = naming_automation('q',indices,'$2nd:\ ','$');

initialize_plot(5,"title",title_str ,"ylab",ylabel_names )
populate_plot(5,[],model.state_constraints(1:5,1)',"reference",true,linestyle='--',color='r')
populate_plot(5,[],model.state_constraints(1:5,2)',"reference",true,linestyle='--',color='r')
populate_plot(5,tmpc,xtraj1(1:5,:)',"display_names",sim1_names,color='b')
populate_plot(5,tmpc,xtraj2(1:5,:)',"display_names",sim2_names,color='k')

