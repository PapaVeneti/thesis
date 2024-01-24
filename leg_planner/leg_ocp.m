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
N = 50;
Th = 5; % time horizon length

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
%% 1. State-input variables:
ocp_model = acados_ocp_model();
ocp_model.set('name', model.name);


ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_xdot', model.sym_xdot);

%% 2. Dynamics:
if (strcmp(sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk irk_gnsf
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.expr_f_impl);
end

%% 3. Constraints:
%0.  Initial State
ocp_model.set('constr_x0', x0);

%1.  Input Box constraints
ocp_model.set('constr_lbu',model.input_constraints(:,1))
ocp_model.set('constr_ubu',model.input_constraints(:,2))
ocp_model.set('constr_Jbu',eye(nu))

%2.  State Box constraints 
%3.  Terminal constraints (on state)
%4.  Path constraints 
%TBD:
%    a. Angular momentum constraints
%    b. Norm constraints on torques

%% 4. Cost:

switch cost_type
    case 'Euclidean'
% Linear LS
ocp_model.set('cost_type','linear_ls')
ocp_model.set('cost_type_e','linear_ls')

% q \in [0-1], w\in[0-10], u \in [0-10]
Q = diag( [100,100,0,100,0,10,10,10,10,10 ] ); 
R = diag( [10,10,10 ] );

Qc = sqrt(Q); %nx * nx %chol
Rc = sqrt(R); %nu * nu
y_ref = [Qc,zeros(nx,nu);zeros(nu,nx),Rc]*[xref;0;0;0] ; %Important to give it like that.

ocp_model.set('cost_Vx', [Qc;zeros(nu,nx)] );
ocp_model.set('cost_Vx_e', Qc );
ocp_model.set('cost_Vu', [zeros(nx,nu);Rc] );
ocp_model.set('cost_W', eye(13) );
ocp_model.set('cost_W_e', eye(10) );

ocp_model.set('cost_y_ref',y_ref)
ocp_model.set('cost_y_ref_e',y_ref(1:nx))

    case 'Geodesic'
        %TBD:

% q = model.sym_x(1:4);
% % cost_expr_ext_cost = quat_mul_SX(qref',reshape(q,1,4));
% cost_expr_y = geodesic_distance_SX(qref,q);
% Q = eye(4);
% 
% % non_linear cost:
% % ocp_model.set('cost_type','nonlinear_ls')
% % ocp_model.set('cost_type_e','nonlinear_ls')
% % 
% % ocp_model.set('cost_expr_y',cost_expr_y);
% % ocp_model.set('cost_expr_y_e',cost_expr_y);
% % 
% % ocp_model.set('cost_W', Q );
% % ocp_model.set('cost_W_e', Q );
% % 
% % ocp_model.set('cost_y_ref',zeros(4,1))
% % ocp_model.set('cost_y_ref_e',zeros(4,1))
% 
% % linear cost:
% ocp_model.set('cost_type','linear_ls')
% ocp_model.set('cost_type_e','linear_ls')
% % 
% % ocp_model.set('cost_expr_y',cost_expr_y);
% % ocp_model.set('cost_expr_y_e',cost_expr_y);
% 
% ocp_model.set('cost_Vx', diag([1000,ones(1,9)])*eye(10,7) );
% ocp_model.set('cost_Vx_e', diag([1000,ones(1,6)])*eye(7) );
% ocp_model.set('cost_Vu', [zeros(7,3);zeros(3)] );
% ocp_model.set('cost_W', eye(10) );
% ocp_model.set('cost_W_e', eye(7) );
% 
% ocp_model.set('cost_y_ref',diag([1000,ones(1,9)])*[qref';zeros(6,1)])
% ocp_model.set('cost_y_ref_e',diag([1000,ones(1,6)])*[qref';zeros(3,1)])

% geodesic_distance_SX(qref,q)
% Adq = quaternion_difference_matrix(qref); 
% Q = eye(4);
% conjI = diag([1,-1,-1,-1]);


% ocp_model.set('cost_W', Q )

% ocp_model.set('cost_Vx_e', [eye(4),zeros(4,3)] )
% ocp_model.set('cost_W_e', (Adq'*Q*Adq) )
% 
% ocp_model.set('cost_y_ref_e',ref(1:end-1))
% ocp_model.set('cost_Vx_e',Qc)
% ocp_model.set('cost_W_e',eye(2))
    otherwise 
        error_msg = ['Wrong cost type.',newline, 'Select `cost_type` from: ["LS","Geodesic"]'];
        error(error_msg)
end

%% 5. Ocp options:
ocp_model.set('T', Th); %Time Horizon

ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('ext_fun_compile_flags', ''); % '-O2'
% ... see ocp_opts.opts_struct to see what other fields can be set

%% 6. Finalize:
ocp = acados_ocp(ocp_model, ocp_opts, simulink_opts);

%% 7. Export to simulink
% make acados simulink files (Optional)
if Do_make_acados_simulink_files
    cd c_generated_code
    make_sfun; % ocp solver
    make_sfun_sim; % integrator
    cd ..
end

y_ref_sim = repmat(y_ref,N-1,1);
lbu_sim = repmat(model.input_constraints(:,1),N,1);
ubu_sim = repmat(model.input_constraints(:,2),N,1);


%% 7. Call solver
q0 = zeros(5,1); 
w0 = zeros(5,1);
x0 = [q0;w0];

%1. update initial state
ocp.set('constr_x0', x0);
ocp.set('constr_lbx', x0, 0)

%2. set trajectory initialization
% ocp.set('init_x', x_traj_init);
% ocp.set('init_u', u_traj_init);
% ocp.set('init_pi', zeros(nx, N))

%3. change values for specific shooting node using:
%   ocp.set('field', value, optional: stage_index)
% ocp.set('constr_lbx', x0, 0)

%4.  solve and get statistics
ocp.solve();
ocp.print('stat')


status = ocp.get('status'); % 0 - success
time_tot = ocp.get('time_tot'); % 0 - success

disp(['Solution status is: ', num2str(status)]);
disp(['Solution time is: ', num2str( 1e3*ocp.get('time_tot'),3 ),'ms' ]);

%% 8. Plot
% get solution
utraj = ocp.get('u');
xtraj = ocp.get('x');

% Xref = [qref,nan,nan,nan];

ts = linspace(0, Th, N+1);
figure("Name","States"); hold on;
States = { '$w$','$q_1$','$q_2$','$q_3$','$\omega_1$','$\omega_2$','$\omega_3$',};
for i=1:length(States)
    subplot(length(States), 1, i);
    plot(ts, xtraj(i,:)); 
    grid on; 
    hold on;
    yline(xref(i),'k--');
    ylabel(States{i},'interpreter','latex',FontSize=25);
    if i < 5
        ylim([-1.1 1.1]);
    end
end
xlabel('t [s]')

figure("Name","Control")
Actuators = {'$\tau_x$','$\tau_y$','$\tau_z$'};
for i=1:length(Actuators)
    subplot(length(Actuators), 1, i);
    stairs(ts(1:end-1), utraj(i,:)); grid on;
    ylabel(Actuators{i},'interpreter','latex',FontSize=25);
    
end
xlabel('t [s]')