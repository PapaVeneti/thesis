%This script creates the dynamics matrices for the simulation of the leg.
%It calculates the Lagrangian and then cals the EL_derivatives.m and
%EL_collect.m functions to create the matrices. 
%
%It reads the dynamics parameters (m,I,CoM) from the URDF, and thus the
%dynamics can be rapidely updated.

%Notes: 
% 1. round_urdf_reading_to_6dec = true;
% 2. vpa for transforms

% REQUIREMENTS:
%1. No fixed joints
%2. Only 1-DOF joints (revolute and prismatic)
%3. 
%% Read robot parameters
robot  = importrobot('leg_description/urdf/ntnu_leg.urdf');
num_bodies = robot.NumBodies;

I  =  zeros(3,3,num_bodies);
rc =  zeros(3,1,num_bodies);
m  =  zeros(1,num_bodies);


for ib = 1:num_bodies
    curr_body = robot.Bodies{ib};
    I (:,:,ib) = matlabInertia2matrix( curr_body.Inertia,curr_body.CenterOfMass,curr_body.Mass);
    rc(:,:,ib) = robot.Bodies{ib}.CenterOfMass';
    m(1,ib)    = robot.Bodies{ib}.Mass;
end

% I(:,:,1) = I1;
% I(:,:,2) = I2;
% I(:,:,3) = I3;

%% Symbolic variables
syms t %symbolic variable for time

state_str= cell(num_bodies,2); %col1: x, col2: x_dot
q  = cell(num_bodies,1); %cell to hold any type of object
qt = cell(num_bodies,1); %cell to hold any type of object


for i=1:num_bodies
    state_str{i,1} = ['q',num2str(i),'(t)'];
    state_str{i,2} = ['q',num2str(i),'t(t)'];

    q{i}  = symfun(state_str{i,1} ,t);
    qt{i} = symfun(state_str{i,2} ,t);

    assumeAlso(q{i}(t) ,'real'); %assumeAlso to keep previous assumptions
    assumeAlso(qt{i}(t),'real');
end


%% Get robot transformations
% assuming body1 is link1

%Transformation to previous link
T = sym('T',[4,4,num_bodies]); 

for ib = 1:num_bodies 
    % Get T_jpj{i}0: Transformation of joint{i} in 0 pos to parent joint jp 
    T0 = robot.Bodies{ib}.Joint.JointToParentTransform;

    % Get T_j{i}0j{i}q: Transformation of joint{i} in q pos to joint{i} in 0
    Tq = jointTF_from_RBjoint(robot.Bodies{ib}.Joint,q{ib});
    
    % T_jpj{i} = T_jpj{i}0 * T_j{i}0j{i}q
    T(:,:,ib) = T0*Tq;
end
robot_chain = Extract_robot_chain(robot);

%transformation to base
T0 = sym('T',[4,4,num_bodies]);


T_wLi = eye(4);
prev_tf_index = 0;
for ib=1:num_bodies


    if (ib ~= 1) && (robot_chain{ib}(2) ~= prev_tf_index)
    %if not 1st body, and the previous body wasn't in the chain, we have to
    %calculate T_wL{i-1}
        T_wLi = eye(4);
        prev_tf_index = robot_chain{ib}(1);
        for irc = 1:length(robot_chain{ib})-1
            T_wLi = T_wLi*T(:,:,robot_chain{ib}(1+irc) );    
        end
    else
        prev_tf_index = ib;
    end
    
  
    T_wLi = T_wLi*T(:,:,ib); 
%     T_wLi = simplify(T_wLi);
    T0(:,:,ib) = T_wLi;

end

%% Positions
xc = sym('xc',[3,num_bodies]);

T_wLi = eye(4);
prev_tf_index = 0;
for ib=1:num_bodies


    if (ib ~= 1) && (robot_chain{ib}(2) ~= prev_tf_index)
    %if not 1st body, and the previous body wasn't in the chain, we have to
    %calculate T_wL{i-1}
        T_wLi = eye(4);
        prev_tf_index = robot_chain{ib}(1);
        for irc = 1:length(robot_chain{ib})-1
            T_wLi = T_wLi*T(:,:,robot_chain{ib}(1+irc) );    
        end
    else
        prev_tf_index = ib;
    end
    
  
    T_wLi = T_wLi*T(:,:,ib); 
%     T_wLi = simplify(T_wLi);
    
    R_wLi = T_wLi(1:3,1:3);
    P_wLi = T_wLi(1:3,4);

    xc(1:3,ib) = P_wLi + R_wLi*rc(:,:,ib) ;
end
% xc = simplify(xc);

%% Velocities


joint= robot.Bodies{1}.Joint;


w  = sym('w' ,[3,num_bodies]); w  = w*0;
u  = sym('u' ,[3,num_bodies]); u  = u*0;
uc = sym('uc',[3,num_bodies]); uc = uc*0;

for ib =1:num_bodies
    R0i   = T0(1:3,1:3,ib); %link_to_base
    Pprevi = T(1:3,4,ib);   %link_to_prev
    
    if ib == 1
        uprev = [0;0;0];
        wprev = [0;0;0];
        R0prev = eye(3);
    else
        iprev = robot_chain{ib}(2);
        %previous    
        wprev = w(:,iprev);
        uprev = u(:,iprev);
        R0prev = T0(1:3,1:3,iprev);

    end

    [wj,uj] = velocities_from_RBjoint(robot.Bodies{ib}.Joint,qt{ib});
    
    %rotational link and CoM:
    wlink = wprev + R0i*wj;

    %linear
    %link velocity
%     u(:,ib)  = uprev + uj + exterior(wprev  )*R0prev*Pprevi;
    ulink = uprev + uj + exterior(wprev  )*R0prev*Pprevi;
    %CoM velocity:
    ucom = ulink    + (exterior(wlink) * R0i   * rc(:,:,ib)); 

    w (:,ib) = simplify(wlink );
    u (:,ib) = simplify(ulink );
    uc(:,ib) = simplify(ucom);


end


%% Kinetic Energy

TKi = sym('TK',[num_bodies,1]); TKi = TKi*0;

for ib = 1:num_bodies
    R = T0(1:3,1:3,ib);

    Tlin = 1/2*m(ib) * uc(:,ib).' *uc(:,ib);
    Trot = 1/2*(w(:,ib).')*R *I(:,:,ib) * (R.') *w(:,ib);
    TKi(ib) = Tlin+Trot;
    TKi(ib) = simplify(TKi(ib));

end



TK = sum(TKi); TK = simplify(TK); 
TK = symfun(TK,t);

%% 
%% Potential Energy
g = 9.8;

% U = [g,0,0]*xc*m'; %my leg
U = [0,0,g]*xc*m'; %ntnu

size(U);

L = symfun( TK-U,t);

%% E-L equations
states_el = sym('s',[2*num_bodies,1]);
for ib=1:num_bodies
    states_el(ib,1)            = q{ib};
    states_el(ib+num_bodies,1) = qt{ib};
end
symfun(states_el,t)


[L_qt,L_q] = EL_derivatives(L,symfun(states_el,t),num_bodies);
[B,C,G]    = EL_collect(L_qt,L_q,symfun(states_el,t),num_bodies);

%% extract
%% Symbolic variables for extraction:

state_out = sym('s',[2*num_bodies,1]);

for ib=1:num_bodies
    state_pos = ['Q',num2str(ib)];
    state_vel = ['Q',num2str(ib),'t'];

    state_out(ib,1)            = sym(state_pos);
    state_out(ib+num_bodies,1) = sym(state_vel);

    assumeAlso(state_out(ib           ,1)  ,'real'); 
    assumeAlso(state_out(ib+num_bodies,1) ,'real');
end




Bs = subs(B,symfun(states_el,t)',state_out');
Cs = subs(C,symfun(states_el,t)',state_out');
Gs = subs(G,symfun(states_el,t)',state_out');

