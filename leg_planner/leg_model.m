function model = leg_model()
%srbd_model: This function creates the struct that contains the dynamic and
%system information for the ntnu leg.
%
% 1. It doesn't include non-linear cost and constraints, as these can be
% defined straight in the ocp. 
%
% 2. It also containts the name of the system.
%

import casadi.*

%% 1.  system dimensions:
nx = 10; 
nu = 3; 

%% 2.  system parameters:
%1. Damping 
KD_definition

%% 3.  symbolic variables:
%state:
q  = SX.sym('q' ,5) ; 
qt = SX.sym('qt',5) ; 

Q1 = q(1);Q1t = qt(1);
Q2 = q(2);Q2t = qt(2);
Q3 = q(3);Q3t = qt(3);
Q4 = q(4);Q4t = qt(4);
Q5 = q(5);Q5t = qt(5);

sym_x    = vertcat(q, qt);
sym_xdot = SX.sym('xdot', nx, 1);

%input: 
tau = SX.sym('tau',nu,1);         % horizontal force acting on cart [N]
sym_u = tau;

%% 4.a dynamics helpers: 
M  = SX.sym('M',5,5);
C  = SX.sym('C',5,5);
G  = SX.sym('G',5,1);
H  = SX.sym('H',2,5);
Ht = SX.sym('Ht',2,5);

%constraint matrix:
HQ  = SX.sym('HQ',2,1);

Dyn = SX.sym('Dyn',nx,1);
matrices
define_functions %finv5,finv2,ftrans5
%% 4.b. dynamics: 
u_gen = [tau(1);tau(2);0;tau(3);0];
D = diag(D);


Minv = finv5(M);
% vel_dyn  = inv(M)* (u_gen - C*qt - G -D*qt);
vel_dyn  = Minv* (u_gen - C*qt - G -D*qt);


%f=  -pinv( H*(  inv(M) *H' ))  *   (H*vel_dyn + Ht*qt);
f=  -finv2( H*Minv*H' )  *   (H*vel_dyn + Ht*qt);


%% 4.b Explicit dynamics:
derivatives = [qt(1);qt(2);qt(3);qt(4);qt(5)];
dynamics    = Minv* (u_gen - C*qt - G + (H')*f -D*qt);
% dynamics    = Minv* (u_gen - C*qt - G -D*qt);


expr_f_expl = vertcat(derivatives,dynamics);

%% 4.c Implicit dynamics (not working):

temp = inv(H*Minv*H');
Nc = eye(5) - Minv*(H')*temp*H ; % constraint null-space matrix


expr_f_impl = [sym_xdot(1:5)-qt;(Nc')*(M*sym_xdot(6:10) + (C+D)*qt + G -u_gen)];

%% 5.  system constraints:
%1. state constraints
state_constraints = zeros(10,2);

%initial but wrong:
% state_constraints(1,:) = [-pi   ,pi  ];
% state_constraints(2,:) = [-pi   ,1.65];
% state_constraints(3,:) = [-2.5  ,1.58];
% state_constraints(4,:) = [-1.65 ,pi  ];
% state_constraints(5,:) = [-1.51 ,2.5 ];

%Revised
state_constraints(1,:) = [-2    ,1.31];
state_constraints(2,:) = [-1.2  ,1.65];   %1.65 to hit MH, 1.2 to not hit the base at full swing of qmh
state_constraints(3,:) = [-1.323,1.42]; %changed for convex shapes (-1.323
state_constraints(4,:) = [-1.65 ,1.2 ];     %1.65 to hit MH, 1.2 to not hit the base at full swing of qmh
state_constraints(5,:) = [-1.42 ,1.323];   %changed for convex shapes


%velocity constraints
max_vel = 32; % [rad/s] from olympian_rl
state_constraints(6:10,:) = max_vel*ones(5,2)*[-1,0;0,1];


%2. input constraints
input_constraints = [10;10;10]*[-1,1];

%3. kinematic loop closure constraint
path_constraints = HQ;

%% 6.  populate structure:
model.name = 'ntnu_leg';

model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.input_constraints = input_constraints;
model.state_constraints = state_constraints;
model.path_constraints = path_constraints;
end