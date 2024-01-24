function model = srbd_model()
%srbd_model: This function creates the struct that contains the dynamic and
%system information for a srbd model for acados OCP. 
%
% 1. It doesn't include non-linear cost and constraints, as these can be
% defined straight in the ocp. 
%
% 2. It also containts the name of the system.
%
% 3. quaternion is: q = [w, q1, q2, q3]

import casadi.*

%% 1.  system dimensions:
nx = 7; %quat 4 + 3 velocities
nu = 3; 

%% 2.  system parameters:
%1. inertia tensor [kg*m^2]
I = zeros(3); 
I(1,2) =-0.00085624;
I(1,3) = 0.0025397;
I(2,3) = 0.00030749;

I = I+I';

I(1,1) = 0.4897949;
I(2,2) = 0.92002036;
I(3,3) = 1.00669705;

%% 3.  symbolic variables:
%state:
q = SX.sym('q',4) ; %quaternions [qx,qy,qz,w]
w = SX.sym('w',3) ; %angular velocity [wx,wy,wz]

sym_x    = vertcat(q, w);
sym_xdot = SX.sym('xdot', nx, 1);

%input: 
tau = SX.sym('tau',nu,1);         % horizontal force acting on cart [N]
sym_u = tau;

%% 4.a dynamics helpers: 
% wquat = [
%     0, w(3), w(2),-w(1);
% -w(3),    0,-w(1),-w(2);
% -w(2), w(1),    0, w(3);
%  w(1), w(2),-w(3),    0;
% ];

wquat = [
    0,-w(1),-w(2),-w(3);
 w(1),    0, w(3),-w(2);
 w(2),-w(3),    0, w(1);
 w(3), w(2),-w(1),    0;
]; 

%2: exterior product of omega
w_ext = [
0 -w(3) w(2);
w(3) 0 -w(1);
-w(2) w(1) 0];

%% 4.b dynamics:
qt = 1/2*wquat*q;
wt = I\ ( tau - w_ext*(I*w) ) ;

expr_f_expl = vertcat(qt,wt);
expr_f_impl = expr_f_expl - sym_xdot;

%% 5.  system constraints:
%1. state constraints

%2. input constraints
input_constraints = [10;10;10]*[-1,1];

%% 6.  populate structure:
model.name = 'SRBD';

model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.input_constraints = input_constraints;

end