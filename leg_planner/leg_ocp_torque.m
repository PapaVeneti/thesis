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
cost_type = 'torque'; %['LS','torque']
solver_statistics = false;
same_init = false;
terminal_constr = 0; 
%0: no tc, 1: pos(1,2,4) + vel, 2:pos(1,2,4), 3 full state tc

% simulink outputs:
simulink_opts.outputs.utraj = 1;
simulink_opts.outputs.xtraj = 1;
simulink_opts.samplingtime = '-1'; %rate of mpc is determined by rate of inputs. Set ZOH before
%% INPUT1: initialization and reference
% initial state:
q0 = zeros(5,1);
q0 = [0;1.45;0;1.1;0];
% w0 = zeros(5,1);
x0 = consistent_x0(q0([1,2,4]),[0;0;0]);

% reference:
qref = [-1;1;0;1;0];
wref = [0;0;0;0;0];
xref = [qref;wref];
xref = x0;

tau_mh_ref = [0.0;0;1];

%torque reference
tau_ref = [0;0;0];
%% INPUT2: discretization-solvers-sim_method
N = 180;
Th = 0.4; % time horizon length

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
slack_pos = 10;
slack_vel = 10;

if strcmp(cost_type,'torque')
slack_pos = 1e-1;
end

if terminal_constr
    switch terminal_constr
        case 1
        %a. pos1,2,4 full vel
        Jbx_e = eye(nx); 
        Jbx_e(5,:) =[]; 
        Jbx_e(3,:) =[]; 
    
        xref_tc = xref; 
        xref_tc(5)= [];
        xref_tc(3)= [];
    
        ns_e  = 8;%number of slack variables for terminal constraint
        Z_e = diag([slack_pos*ones(3,1);slack_vel*ones(5,1)]);
    
        case 2
        %b. pos1,2,4
        Jbx_e = [eye(3),zeros(3,10)];
        xref_tc = xref([1,2,4]);
    
        ns_e = 3;
        Z_e = diag(slack_pos*ones(3,1));
        
        
        case 3
        %c. full state
        Jbx_e = eye(nx); 
        xref_tc = consistent_x0(xref([1,2,4]),xref([6,7,9]));

        ns_e  = 10;
        Z_e = diag([slack_pos*ones(5,1);slack_vel*ones(5,1)]);
    
        otherwise 
    
    end

    %Set terminal constraint
    ocp_model.set('constr_Jbx_e',Jbx_e)
    ocp_model.set('constr_lbx_e',xref_tc)
    ocp_model.set('constr_ubx_e',xref_tc)
    
    %Set slack variables for terminal constraint
    ocp_model.set('constr_Jsbx_e',eye(ns_e));
    ocp_model.set('cost_Z_e',Z_e);
    ocp_model.set('cost_z_e',ones(ns_e,1));
    
end

%4.  Path constraints 
%TBD:
n_pc = 2; %number of path constraints
constraint_coefficients;
C_c      = zeros(n_pc,nx);
D_c      = zeros(n_pc,nu);
C_c(1,:) = [cc_1,zeros(1,nx/2)];
C_c(2,:) = [cc_2,zeros(1,nx/2)];

upper_g = [cb_1;cb_2];
lower_g = [-1e5;-1e5];

% % rate constraints
% C_r  = zeros(N-1,nx);
% D_r  = eye(N) - circshift(eye(N),-1);
% D_r(N,:)=[];
% DULIM = (10*0.25)*(Th/N)*ones(N-1,1);
% 
% %cummulative path constraints:
% C_p = [C_c;C_r];
% D_p = [D_c;D_r];


ocp_model.set('constr_C',C_c);
ocp_model.set('constr_D',D_c);
ocp_model.set('constr_ug',upper_g);
ocp_model.set('constr_lg',lower_g); %cannot handle one sided constraints



%    a. Angular momentum constraints
%    b. Norm constraints on torques

%% 4. Cost:

switch cost_type
    case 'LS'
% Linear LS
ocp_model.set('cost_type','linear_ls')
ocp_model.set('cost_type_e','linear_ls')

Q = diag( [100,100,0,100,0,10,10,10,10,10 ] ); 
R = diag( [10,10,10 ] );

Qc = sqrt(Q); %nx * nx %chol
Rc = sqrt(R); %nu * nu
y_ref = [Qc,zeros(nx,nu);zeros(nu,nx),Rc]*[xref;tau_ref] ; %Important to give it like that.

ocp_model.set('cost_Vx'  , [Qc;zeros(nu,nx)] );
ocp_model.set('cost_Vx_e', Qc                );
ocp_model.set('cost_Vu'  , [zeros(nx,nu);Rc] );
ocp_model.set('cost_W'   , eye(nx+nu)        );
ocp_model.set('cost_W_e' , eye(nx)           );

ocp_model.set('cost_y_ref'  ,y_ref      )
ocp_model.set('cost_y_ref_e',y_ref(1:nx))

    case 'torque'
ocp_model.set('cost_type','nonlinear_ls')
% non linear torque cost term:
qmh    = model.sym_x(1);
tau    = model.sym_u;
y_expr = [tau(1);( tau(2)+tau(3) ) *[-sin(qmh);cos(qmh)]];

ocp_model.set('cost_expr_y',y_expr)
ocp_model.set('cost_y_ref' ,tau_mh_ref)
Rweight = abs( tau_mh_ref/norm(tau_mh_ref) )';
ocp_model.set('cost_W'   , diag( [1,1,10])/25);

%linear mayer cost term:
ocp_model.set('cost_type_e','linear_ls')

Q = diag( [100,100,100,100,100,10,10,10,10,10 ] );
% Q = 10*diag( ones(1,10));

Qc = sqrt(Q); %nx * nx %chol
y_ref = Qc*x0 ;
ocp_model.set('cost_y_ref_e',y_ref)
ocp_model.set('cost_W_e' , eye(nx));
ocp_model.set('cost_Vx_e', Qc);

    otherwise 
        error_msg = ['Wrong cost type.',newline, 'Select `cost_type` from: ["LS"]'];
        error(error_msg)
end

%% 5. Ocp options:


ocp_model.set('T', Th); %Time Horizon

ocp_opts = acados_ocp_opts();
ocp_opts.set('qp_solver_iter_max', 1000); %usually goes max 15-20 -> if 50 it fails too with tc
ocp_opts.set('nlp_solver_max_iter', 100);
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('ext_fun_compile_flags', ''); % '-O2'

%extra options:
%globalization: 
% ocp_opts.set('levenberg_marquardt', 1e-2);
% ocp_opts.set('globalization', 'MERIT_BACKTRACKING');
% ocp_opts.set('alpha_min', 0.05);
% ocp_opts.set('alpha_min', 100);

%fidelity:
ocp_opts.set('sim_method_num_steps', 1); %Default 1
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

%1. update initial state
ocp.set('constr_x0', x0);
ocp.set('constr_lbx', x0, 0)
% 
if strcmp(cost_type,'torque')
    Ncutoff = floor( N*0.75);
    Nrest   = N-Ncutoff;
    treturn = -tau_mh_ref*(Ncutoff/Nrest);
    tref_plot = zeros(N,nu);

    
    x_traj_init = zeros(nx,N+1);
    x_retrun = consistent_x0([x0(1),-x0(2),-x0(4)],[0,0,0]);
    steps = Ncutoff ;                             %// number of steps
    x_traj_init(:,1:Ncutoff) = bsxfun(@plus,((x_retrun(:)-x0(:))./(steps-1))*[0:steps-1],x0(:));
    steps = Nrest+1 ;
    x_traj_init(:,Ncutoff+1:end) = bsxfun(@plus,((x0(:)-x_retrun(:))./(steps-1))*[0:steps-1],x_retrun(:));

    for iN = 0:N-1
        if iN< Ncutoff
            tref_current = tau_mh_ref;
        else
            tref_current = treturn;
        end
        tref_plot(iN+1,:) = tref_current';
        ocp.set('cost_y_ref',tref_current ,iN)
    end
end
% 


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
%% 7a. Solver statistics:
TESTS = 2;
NTEST = 100;
status = 4; %init guess
xref_new = xref;


if solver_statistics 
    SOLTIME   = nan(TESTS*NTEST,4); %4 time statistics
    SOLSTATUS = ones(TESTS*NTEST,1);

    for it = 1:TESTS   
        disp(['TEST no:',num2str(it)])
        pause(1)
        if it == 2
%             xref_new  = [-0.3,1.2,0,1.32,0];
%             xref_new  = [-0.5,-1.5,0,-1.5,0]; %out of bounds for new constraints
            xref_new  = [-0.5,-0.99,0,-1.5,0];
            xref_new  = [xref_new';0;0;0;0;0];
            y_ref_new = [Qc,zeros(nx,nu);zeros(nu,nx),Rc]*[xref_new;tau_ref] ; %Important to give it like that.
            ocp.set('cost_y_ref', y_ref_new);
            ocp.set('cost_y_ref_e', y_ref_new(1:nx));
            
            %Update terminal constr
            if terminal_constr
                switch terminal_constr
                    case 1
                    %terminal constraint
                    xref_tc = xref_new; 
                    xref_tc(5)= [];xref_tc(3)= [];

                    case 2
                    xref_tc = xref_new([1 2 4]);     

                    case 3
                    xref_tc = consistent_x0(xref_new([1,2,4]),xref_new([6,7,9]));

                end
                ocp.set('constr_lbx',xref_tc,N)
                ocp.set('constr_ubx',xref_tc,N)
            end
        end
    
        %consistent final state:
        xref_for_init = consistent_x0(xref_new([1,2,4]),xref_new([6,7,9]));
        xinit =zeros(nx,N+1);

        for ii =1:NTEST
            
            %disturb the initial conditions:
            random_q0 =   2e-1*rand(3,1);
            random_w0 =   5*   rand(3,1);
            
            %random initial condtions for q1,q2,q4
            q0r = q0([1,2,4]) +  random_q0;
            w0r = w0([1,2,4]) +  random_w0;    
            
            %consistent initial conditions
            random_x0 = consistent_x0(q0r,w0r);
            
            %1. update initial state
            ocp.set('constr_x0', random_x0);
            ocp.set('constr_lbx',random_x0, 0)
            ocp.set('constr_ubx',random_x0, 0)

            %initialization
            if status == 4 || same_init
                ocp.set('init_x',zeros(10,N+1))
                ocp.set('init_u',zeros(3,N))
                
                for iS = 1:10
                    xinit(iS,:) = linspace(random_x0(iS),xref_for_init(iS),N+1);
                end
                ocp.set('init_x',xinit)
            end

            
            %2.  solve and get statistics
            ocp.solve();
            ocp.print('stat')
                        
            status = ocp.get('status'); % 0 - success
            time_tot = ocp.get('time_tot'); % 0 - success
            
            disp(['Solution status is: ', num2str(status)]);
            disp(['Solution time is: ', num2str( 1e3*ocp.get('time_tot'),3 ),'ms' ]);
            
            
            if status == 4
                error('')
            end
            if status == 0 || status == 2
                %max iter -> still a solution, succussful
                time_stats(1,1) = ocp.get('time_tot');
                time_stats(1,2) = ocp.get('time_lin');
                time_stats(1,3) = ocp.get('time_qp_sol');
                time_stats(1,4) = ocp.get('time_reg');

                SOLTIME((it-1)*NTEST+ ii,:)   = time_stats;

            else
                SOLTIME((it-1)*NTEST+ ii,:)   = nan*zeros(1,4);
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
    mean_sol_time = mean(SOLTIME(:,1),'omitnan');
    std_sol_time = std (SOLTIME(:,1),'omitnan');
    disp(['Mean solution time is: ', num2str( 1e3*mean_sol_time,3 ),'ms']);
    disp(['Standard deviation is: ', num2str( 1e3*std_sol_time ,3 ),'ms']);
    %linearization
    mean_lin_time = mean(SOLTIME(:,2),'omitnan');
    std_lin_time = std (SOLTIME(:,2),'omitnan');
    disp('--------------')
    disp(['Mean linearization time is: ', num2str( 1e3*mean_lin_time,3 ),'ms']);
    disp(['Standard deviation is: ', num2str( 1e3*std_lin_time ,3 ),'ms']);

    end
end

%% 8. Plot
% get solution
utraj = ocp.get('u');
xtraj = ocp.get('x');

% Xref = [qref,nan,nan,nan];

%% visualize trajectoryy:
tmpc    = linspace(0,Th,N+1)';
qin =  [tmpc,xtraj(1:5,:)'];
win =  [tmpc,xtraj(6:10,:)'];
out = sim('simulink/ntnu_leg_motion_generation.slx',tmpc(end));

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

%% Comparison plots: [t_MH]
figure(Name= 'Comparison: [u]')
mh_index = {'X','Y','Z'};

title_str   = '$Acados \ optimized\ trajectory:\ [\tau_{MH}] $';
ylabel_names = naming_automation('\tau',mh_index,'$','$');
ref_names    = naming_automation('\tau',mh_index,'$reference:\ ','$');
sim_names    = naming_automation('\tau',mh_index,'$sim:\ ','$');
simu_names   = naming_automation('\tau',mh_index,'$simulink:\ ','$');

TMH = base_torque_calc(xtraj(1,1:end-1)',utraj');

initialize_plot(3,"title",title_str ,"ylab",ylabel_names )
% populate_plot(3,tmpc(1:end-1),tref_plot,linestyle='--',color='g',display_names=ref_names)
% populate_plot(3,tmpc(1:end-1),ones(N,1)*tref_plot',linestyle='--',color='g',display_names=ref_names)
populate_plot(3,tmpc(1:end-1),TMH,"display_names",sim_names)


%%
% load handel
% sound(y,Fs)
