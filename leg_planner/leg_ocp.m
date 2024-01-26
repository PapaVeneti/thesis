%% acados prerequisites
% 1. Create the struct that containts the simulink export options.
% Modify `simulink_opts`.
if ~exist('simulink_opts')
    disp('using acados simulink default options')
    simulink_opts = get_acados_simulink_opts;
end

% 2.
check_acados_requirements()

%% INPUT0: ocp definition options - Simulink outputs
Do_make_acados_simulink_files = false;
cost_type = 'Euclidean'; %['LS']
solver_statistics = true;
same_init = true;
terminal_constr = true;

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
qref = [-1;1;0;1;0];
wref = [0;0;0;0;0];
xref = [qref;wref];

%torque reference
tau_ref = [0;0;0];
%% INPUT2: discretization-solvers-sim_method
N = 20;
Th = 1.5; % time horizon length

nlp_solver = 'sqp'; % Choose from ['sqp', 'sqp_rti']
qp_solver = 'partial_condensing_hpipm';
% Choose from: 
% 1. 'full_condensing_hpipm', 
% 2. 'partial_condensing_hpipm', 
% 3. 'full_condensing_qpoases', 
% 4. 'full_condensing_daqp'
qp_solver_cond_N =  N; % for partial condensing
sim_method = 'erk'; % integrator type. Choose from ['erk','irk','irk_gnsf']

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
ocp_model.set('constr_lbx',model.state_constraints(:,1))
ocp_model.set('constr_ubx',model.state_constraints(:,2))
ocp_model.set('constr_Jbx',eye(nx))


%3.  Terminal constraints (on state)
if terminal_constr
    Jbx_e = eye(nx); 
    Jbx_e(5,:) =[]; 
    Jbx_e(3,:) =[];  
    xref_tc = xref; 
    xref_tc(5)= [];xref_tc(3)= [];
    ocp_model.set('constr_Jbx_e',Jbx_e)
    ocp_model.set('constr_lbx_e',xref_tc)
    ocp_model.set('constr_ubx_e',xref_tc)

    % slack variable:
    ocp_model.set('constr_Jsbx_e',eye(8));
    ocp_model.set('cost_Zl_e',eye(8));
    ocp_model.set('cost_Zu_e',eye(8));
    ocp_model.set('cost_z_e',zeros(8,1));
%     ocp_model.set('cost_Z_e',eye(8))
end

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
y_ref = [Qc,zeros(nx,nu);zeros(nu,nx),Rc]*[xref;tau_ref] ; %Important to give it like that.

ocp_model.set('cost_Vx', [Qc;zeros(nu,nx)] );
ocp_model.set('cost_Vx_e', Qc );
ocp_model.set('cost_Vu', [zeros(nx,nu);Rc] );
ocp_model.set('cost_W', eye(nx+nu) );
ocp_model.set('cost_W_e', eye(nx) );

ocp_model.set('cost_y_ref',y_ref)
ocp_model.set('cost_y_ref_e',y_ref(1:nx))

    case 'Geodesic'

    otherwise 
        error_msg = ['Wrong cost type.',newline, 'Select `cost_type` from: ["LS"]'];
        error(error_msg)
end

%% 5. Ocp options:


ocp_model.set('T', Th); %Time Horizon

ocp_opts = acados_ocp_opts();
ocp_opts.set('qp_solver_iter_max', 200);
ocp_opts.set('nlp_solver_max_iter', 100);

ocp_opts.set('sim_method_num_steps', 1); %Default 1

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

% bx_e_sim = repmat(xref_tc,N,1);
% ubx_e_sim = repmat(xref_tc,N,1);



%% 7. Call solver
% q0 = zeros(5,1); 
% w0 = zeros(5,1);
% x0 = [q0;w0];
% 
% %1. update initial state
% ocp.set('constr_x0', x0);
% ocp.set('constr_lbx', x0, 0)
% 
% %2. set trajectory initialization
% % ocp.set('init_x', x_traj_init);
% % ocp.set('init_u', u_traj_init);
% % ocp.set('init_pi', zeros(nx, N))
% 
% %3. change values for specific shooting node using:
% %   ocp.set('field', value, optional: stage_index)
% % ocp.set('constr_lbx', x0, 0)
% 
% %4.  solve and get statistics
% ocp.solve();
% ocp.print('stat')
% 
% 
% status = ocp.get('status'); % 0 - success
% time_tot = ocp.get('time_tot'); % 0 - success
% 
% disp(['Solution status is: ', num2str(status)]);
% disp(['Solution time is: ', num2str( 1e3*ocp.get('time_tot'),3 ),'ms' ]);
%% 7a. Solver statistics:
TESTS = 2;
NTEST = 100;
status = 4; %init guess


if solver_statistics 
    SOLTIME   = nan(TESTS*NTEST,1);
    SOLSTATUS = ones(TESTS*NTEST,1);

    for it = 1:TESTS   
        disp(['TEST no:',num2str(it)])
        pause(1)
        if it == 2
%             xref_new  = [-0.3,1.2,0,1.32,0];
            xref_new  = [-0.5,-1.5,0,-1.5,0];
            xref_new  = [xref_new';0;0;0;0;0];
            y_ref_new = [Qc,zeros(nx,nu);zeros(nu,nx),Rc]*[xref_new;tau_ref] ; %Important to give it like that.
            ocp.set('cost_y_ref', y_ref_new);
            ocp.set('cost_y_ref_e', y_ref_new(1:nx));

            if terminal_constr
                %terminal constraint
                xref_tc = xref_new; 
                xref_tc(5)= [];xref_tc(3)= [];

                ocp.set('constr_lbx',xref_tc,N)
                ocp.set('constr_ubx',xref_tc,N)
            end
        end

        for ii =1:NTEST
            
            %disturb the initial conditions:
            random_q0 = 2e-1*rand(3,1);
            random_w0 =   5*   rand(3,1);
            
            %random initial condtions for q1,q2,q4
            q0r = q0([1,2,4]) +  random_q0;
            w0r = w0([1,2,4]) +  random_w0;    
            
            %consistent initial conditions
            random_x0 = consistent_x0(q0r,w0r);
            
            %1. update initial state
            ocp.set('constr_x0', random_x0);
            ocp.set('constr_lbx',random_x0, 0)

            %initialization
            if status == 4 || same_init
                ocp.set('init_x',zeros(10,N+1))
                ocp.set('init_u',zeros(3,N))
            end

            
            %2.  solve and get statistics
            ocp.solve();
            ocp.print('stat')
                        
            status = ocp.get('status'); % 0 - success
            time_tot = ocp.get('time_tot'); % 0 - success
            
            disp(['Solution status is: ', num2str(status)]);
            disp(['Solution time is: ', num2str( 1e3*ocp.get('time_tot'),3 ),'ms' ]);
            
            

            if status == 0 || status == 2
                %max iter -> still a solution, succussful
                SOLTIME((it-1)*NTEST+ ii)   = ocp.get('time_tot');
            else
                SOLTIME((it-1)*NTEST+ ii)   = nan;
            end
            SOLSTATUS((it-1)*NTEST+ ii) = ocp.get('status');
        end
    

    failures = length(find(SOLSTATUS ~= 0));
    
    disp(['SUCCESSFUL MPC: '  , num2str(length(find(SOLSTATUS == 0)))]);
    disp(['UNSUCCESSFUL MPC: ', num2str(failures)]);
    if failures ~= 0
        disp(['NAN_DETECTED: '    , num2str(length(find(SOLSTATUS == 1)))]);
        disp(['MAXITER: '         , num2str(length(find(SOLSTATUS == 2)))]);
        disp(['MINSTEP: '         , num2str(length(find(SOLSTATUS == 3)))]);
        disp(['QP_FAILURE: '      , num2str(length(find(SOLSTATUS == 4)))]);
    end
    mean_sol_time = mean(SOLTIME,'omitnan');
    std_sol_time = std (SOLTIME,'omitnan');
    disp(['Mean solution time is: ', num2str( 1e3*mean_sol_time,3 ),'ms']);
    disp(['Standard deviation is: ', num2str( 1e3*std_sol_time ,3 ),'ms']);

num2str( 1e3*mean_sol_time,3 )
    end
end

%% 8. Plot
% get solution
utraj = ocp.get('u');
xtraj = ocp.get('x');

% Xref = [qref,nan,nan,nan];





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
populate_plot(5,[],xref_plot(1:5)',"reference",true,linestyle='--',color='g',display_names=ref_names)
populate_plot(5,tmpc,xtraj(1:5,:)',"display_names",sim_names)


% populate_plot(5,tsimulink,q_sim,"color",'b',linestyle='--',display_names=simu_names)

%% Comparison plots: [w]
figure(Name= 'Comparison: [w]')

title_str   = '$Acados \ optimized\ trajectory:\ [\omega] $';
ylabel_names = naming_automation('\omega',indices,'$','$');
sim_names    = naming_automation('\omega',indices,'$sim:\ ','$');
simu_names   = naming_automation('\omega',indices,'$simulink:\ ','$');

initialize_plot(5,"title",title_str ,"ylab",ylabel_names )
populate_plot(5,[],xref_plot(6:10)',"reference",true,linestyle='--',color='g',display_names=ref_names)
populate_plot(5,tmpc,xtraj(6:10,:)',"display_names",sim_names)

% populate_plot(5,tsimulink,q_sim,"color",'b',linestyle='--',display_names=simu_names)



%% Comparison plots: [u]
figure(Name= 'Comparison: [u]')

title_str   = '$Acados \ optimized\ trajectory:\ [\tau] $';
ylabel_names = naming_automation('\tau',indices([1,2,4]),'$','$');
sim_names    = naming_automation('\tau',indices([1,2,4]),'$sim:\ ','$');
simu_names   = naming_automation('\tau',indices([1,2,4]),'$simulink:\ ','$');

initialize_plot(3,"title",title_str ,"ylab",ylabel_names )
populate_plot(3,tmpc(1:end-1),utraj',"display_names",sim_names)

% populate_plot(5,tsimulink,q_sim,"color",'b',linestyle='--',display_names=simu_names)


%%
load handel
sound(y,Fs)
