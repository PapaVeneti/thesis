%% acados prerequisites
% 1. Create the struct that containts the simulink export options.
% Modify `simulink_opts`.
if ~exist('simulink_opts')
    disp('using acados simulink default options')
    simulink_opts = get_acados_simulink_opts;
end

% 2.
check_acados_requirements()

% 3. add helper funcs to current session
% sys_path = path; 
% helper_dir = [pwd,'/helper_funcs'];
% if strfind(sys_path,helper_dir)
%     disp('Helper functions are added to path');
% else
%     addpath("helper_funcs/") %for current session
% end
% clear sys_path  helper_dir;

%% INPUT0: ocp definition options - Simulink outputs
Do_make_acados_simulink_files = false;
cost_type = 'Euclidean'; %['LS']

% simulink outputs:
simulink_opts.outputs.utraj = 1;
simulink_opts.outputs.xtraj = 1;
simulink_opts.samplingtime = '-1'; %rate of mpc is determined by rate of inputs. Set ZOH before
%% INPUT1: initialization and reference
% initial state:
q0 = zeros(5,1); 
w0 = zeros(5,1);
x0 = [q0;w0];

% reference:
qref = [1;1;0;1;0];
wref = [0;0;0;0;0];
xref = [qref;wref];
%% INPUT2: discretization-solvers-sim_method
Th = 0.001; % time horizon length
simulation_time = 10;
N = simulation_time/Th;

nlp_solver = 'sqp'; % Choose from ['sqp', 'sqp_rti']
qp_solver = 'partial_condensing_hpipm';
% Choose from: 
% 1. 'full_condensing_hpipm', 
% 2. 'partial_condensing_hpipm', 
% 3. 'full_condensing_qpoases', 
% 4. 'full_condensing_daqp'
qp_solver_cond_N = N; % for partial condensing
sim_method = 'irk'; % integrator type. Choose from ['erk','irk','irk_gnsf']

%% model dynamics [missing ny]
model = leg_model;
nx = model.nx;
nu = model.nu;
% ny = size(model.cost_expr_y, 1);      % used in simulink example
% ny_e = size(model.cost_expr_y_e, 1);

%% 0. Formulation: 
method = 'irk_gnsf'; % irk, irk_gnsf
%% 1. State-input variables:
sim_model = acados_sim_model();
sim_model.set('name', model.name);

sim_model.set('sym_x', model.sym_x);
sim_model.set('sym_u', model.sym_u);
sim_model.set('sym_xdot', model.sym_xdot);

%% 2. Dynamics:
if (strcmp(sim_method, 'erk'))
    sim_model.set('dyn_type', 'explicit');
    sim_model.set('dyn_expr_f', model.expr_f_expl);
else % irk irk_gnsf
    sim_model.set('dyn_type', 'implicit');
    sim_model.set('dyn_expr_f', model.expr_f_impl);
end

%% 3. Constraints: (no need)
% %0.  Initial State
% sim_model.set('constr_x0', x0);
% 
% %1.  Input Box constraints
% sim_model.set('constr_lbu',model.input_constraints(:,1))
% sim_model.set('constr_ubu',model.input_constraints(:,2))
% sim_model.set('constr_Jbu',eye(nu))

%% 4. Sim options:
sim_model.set('T', Th); %Time Horizon

sim_opts = acados_sim_opts();
sim_opts.set('compile_interface', 'auto');
sim_opts.set('num_stages', 1);
sim_opts.set('num_steps', 1);
sim_opts.set('newton_iter', 2); % for implicit intgrators
sim_opts.set('method', 'irk');
sim_opts.set('sens_forw', 'true'); % generate forward sensitivities
if (strcmp(method, 'irk_gnsf'))
	sim_opts.set('gnsf_detect_struct', 'true');
end
    
%% 6. Finalize:
sim = acados_sim(sim_model, sim_opts);

%% 7. Call solver
q0 = zeros(5,1); 
w0 = zeros(5,1);
x0 = [q0;w0];


XS = zeros(10,N+1);
XS(:,1) = x0;



%2. set trajectory initialization
% ocp.set('init_x', x_traj_init);
% ocp.set('init_u', u_traj_init);
% ocp.set('init_pi', zeros(nx, N))

%3. change values for specific shooting node using:
%   ocp.set('field', value, optional: stage_index)
% ocp.set('constr_lbx', x0, 0)

%4.  solve and get statistics
sim.set('u',  [0;0;0]);

for i=1:N
    %1. update initial state
    sim.set('x', XS(:,i));

    sim.solve();

    XS(:,i+1) =  sim.get('x');

end

%% Custom sim
sim_time = [0 simulation_time];
qinitial   = q0;
qinitial_t = w0;
KD_definition

opts = odeset(AbsTol=1e-6,RelTol=1e-3);
controlled_leg = @(t,y) ntnu_leg_closed([],y,[0;0;0],D );
[t,y] = ode45(@(t,y) controlled_leg(t,y),sim_time,[qinitial;qinitial_t],opts);

%% Comparison plots: [q]
indices = {'MH','11','21','12','22'};
tq    = linspace(0,simulation_time,N+1);
% q_sim = out.q.Data;
% w_sim = out.w.Data;

figure(Name= 'Comparison: [q]')
title_str   = '$Comparison\ between\ custom\ simulation\ and\ acados:\ [q] $';
ylabel_names = naming_automation('q',indices,'$','$');
sim_names    = naming_automation('q',indices,'$sim:\ ','$');
simu_names   = naming_automation('q',indices,'$acados:\ ','$');

initialize_plot(5,"title",title_str ,"ylab",ylabel_names )
populate_plot(5,t,y(:,[1 2 3 4 5]),"display_names",sim_names)
populate_plot(5,tq,XS(1:5,:)',"color",'b',linestyle='--',display_names=simu_names)





status = ocp.get('status'); % 0 - success
time_tot = ocp.get('time_tot'); % 0 - success

disp(['Solution status is: ', num2str(status)]);
disp(['Solution time is: ', num2str( 1e3*ocp.get('time_tot'),3 ),'ms' ]);

