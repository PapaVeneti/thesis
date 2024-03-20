%% simparameters
quadruped_params_init
%% BODY PLANNER : initialization and setup
% initial state:
q0_b = [1; 0; 0;0]; 
w0_b = [0;0;0];
x0_b = [q0_b;w0_b];

Nb = 50;
Thb = 5; % time horizon length
nx_b = 7;
nu_b = 3;
srbd_model_obj = srbd_model;

sampling_body = 2e-2;
gravity = [0,0,0]; 

%% BODY PLANNER : reference (and cost)
eul_ref = [0,pi,0];
qref_b = eul2quat(eul_ref,'ZYX')';
wref_b = [0;0;0];
xref_b = [qref_b;wref_b];

% q \in [0-1], w\in[0-10], u \in [0-10]
Q = diag( [100,100,100,100,10,10,10 ] ); 
R = diag( [10,10,10 ] );
Qc = chol(Q); %nx_b * nx_b
Rc = chol(R); %nu_b * nu_b

yref_body = [Qc,zeros(nx_b,nu_b);zeros(nu_b,nx_b),Rc]*[xref_b;0;0;0] ; %Important to give it like that.

%export for simulink
y_ref_body  = repmat(yref_body,Nb-1,1);
lbu_body    = repmat(srbd_model_obj.input_constraints(:,1),Nb,1);
ubu_body    = repmat(srbd_model_obj.input_constraints(:,2),Nb,1);

%% BODY PLANNER - LEG PLANNER interface

Rinterface = eul2rotm([0,0,-pi/2]);


%% LEG PLANNER : initialization and setup

reference_mode = 'pitch';

switch reference_mode
    case 'roll'
        % initial state:
        q0 = [1;0;0;0;0];
        x0 = consistent_x0(q0([1,2,4]),[0;0;0]);
        xref =[];

        %rep1
%         x0  = [ -0.9111   -0.5096    0.9842    1.0809   -1.0948         0         0         0         0         0]';

        %roll reference:
        tau_mh_ref = [-0.2;0;0]; %From Body to Leg
        Wtrack     = [1, 0.1,0.25];
%         Wtrack     = [0.1, 0.1,0.1];


       % Th = 1s
        
    case 'pitch'
        % initial state:
%         q0 = [0;1.45;0;1.1;0]; %initial
        q0_leg = [0;1.45;0;1;0]; %initial
        w0_leg = [0;5;5];

        x0_leg = consistent_x0(q0_leg([1,2,4]),w0_leg);

        %pitch reference:
        tau_mh_ref = [0.0;0;1]; %From Body to Leg
%         Wtrack     = [0.1, 0.2,1]; %for gravity
        Wtrack     = [0.2, 0.01,1];

%gravity 
%         x0 =[0.7907   -0.9357   -0.1250   -1.5430    0.9750         0         0         0         0         0]';
%         x0 =[0.0231   -0.9027   -0.1319   -1.5104    0.9615         0         0         0         0         0]';
%         %Th = 0.75s

    case 'pos'
        % reference:
        qref = [-1;1;0;1;0];
        wref = [0;0;0;0;0];
        xref = [qref;wref];
        x0 = consistent_x0(q0([1,2,4]),[0;0;0]);
        xref = x0;
end

%torque reference
tau_ref = [0;0;0];

%% add a single leg in the simulation to see it
Kp =   1;
Kd = 0.2;
% 
% Kp = 2;
% Kd = 0.4;
KD_definition
constraint_coefficients
leg_model_obj = leg_model;
Nl = 15;
Thl = 0.1;
nu_l = 3;
nx_l = 10;
size_of_ref = 7;
output_size = size_of_ref*(Nl-1);

%roll limits
% leg_model_obj.state_constraints(1,2) = 0.8; %for roll - pitch

% %export for simulink
lbu_leg   = repmat(leg_model_obj.input_constraints(:,1),Nl,1);
ubu_leg   = repmat(leg_model_obj.input_constraints(:,2),Nl,1);
lbx_leg   = repmat(leg_model_obj.state_constraints(:,1),Nl-1,1);
ubx_leg   = repmat(leg_model_obj.state_constraints(:,2),Nl-1,1);

%general linear constraints
upper_g = [cb_1;cb_2];
lower_g = [-1e5;-1e5];
lg_leg   = repmat(lower_g,Nl,1);
ug_leg   = repmat(upper_g,Nl,1);

%path constraints
lh_leg     = repmat([0;0],Nl,1);
uh_leg     = repmat([0;0],Nl,1);
lh_e_leg   = [0;0];
uh_e_leg   = [0;0];

%% add a whole quadruped

