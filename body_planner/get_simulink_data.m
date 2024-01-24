Nmpc = size(out.u_traj.Data,1);

state_figure = figure("Name","States"); 
control_figure = figure("Name","Control");

%% 
for impc = 1:Nmpc
    disp_text = ['Current solution is:  ', num2str(impc), ' from ',num2str(Nmpc)];
    display(disp_text)

t_sim = linspace(0,Th,N+1);
u_sim = unpack_var(out.u_traj.Data(impc,:),3);
x_sim = unpack_var(out.x_traj.Data(impc,:),7);
ts = linspace(0, Th, N+1);



figure(state_figure)
States = { '$w$','$q_1$','$q_2$','$q_3$','$\omega_1$','$\omega_2$','$\omega_3$',};
for i=1:length(States)
    subplot(length(States), 1, i);
    plot(ts, x_sim(i,:)); 
    yline(xref(i),'k--');
    ylabel(States{i},'interpreter','latex',FontSize=25);
    if i < 5
        ylim([-1.1 1.1]);
    end
end
xlabel('t [s]')
hold off

figure(control_figure)
Actuators = {'$\tau_x$','$\tau_y$','$\tau_z$'};
for i=1:length(Actuators)
    subplot(length(Actuators), 1, i);
    stairs(ts(1:end-1), u_sim(i,:)); grid on;
    ylabel(Actuators{i},'interpreter','latex',FontSize=25);
end
xlabel('t [s]')
hold off


pause(0.1)


end

