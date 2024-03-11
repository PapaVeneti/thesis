Nl = 30;N=30;
KD_definition
constraint_coefficients
gravity = [0;0;0];
nx = 10; nx_l = 10;
nu = 3; nu_l = 3;
Th = 1.5;

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


Q = diag( [100,100,0,100,0,10,10,10,10,10 ] ); 
R = diag( [10,10,10 ] );

Qc = sqrt(Q); %nx * nx %chol
Rc = sqrt(R); %nu * nu
y_ref = [Qc,zeros(nx,nu);zeros(nu,nx),Rc]*[xref;tau_ref] ; %

%% ref 

leg_model_obj = leg_model;

y_ref_sim = repmat(y_ref,N-1,1);
lbu_sim = repmat(leg_model_obj.input_constraints(:,1),N,1);
ubu_sim = repmat(leg_model_obj.input_constraints(:,2),N,1);

% %export for simulink
lbu_leg   = repmat(leg_model_obj.input_constraints(:,1),Nl,1);
ubu_leg   = repmat(leg_model_obj.input_constraints(:,2),Nl,1);
lbx_leg   = repmat(leg_model_obj.state_constraints(:,1),Nl-1,1);
ubx_leg   = repmat(leg_model_obj.state_constraints(:,2),Nl-1,1);
lbx_e_leg   = xref;
ubx_e_leg   = xref;

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