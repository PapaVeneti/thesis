%This script creates the constraints matrices H, Ht for the dynamic
%simulation of the leg.
%
%More info here: https://underactuated.csail.mit.edu/multibody.html#bilateral

%% state vectors:
qs  = sym('s',[num_bodies,1]);
qts = sym('s',[num_bodies,1]);
for ib=1:num_bodies
    qs(ib,1)  = q{ib};
    qts(ib,1) = qt{ib};
end
qs  = symfun(qs,t);
qts = symfun(qts,t);


%% Constraint Definition
TEE_1 = [eye(3),[0.29977 ;0;0];0 0 0 1];
TEE_2 = [eye(3),[0.29929 ;0;0];0 0 0 1];

TA =  T(:,:,2)* T(:,:,3)*TEE_1;  
TB =  T(:,:,4)* T(:,:,5)*TEE_2;

% Constraint: Index 3: h(q) = 0
hq = symfun( TA(1:3,4) - TB(1:3,4),t);

% Simplify constraint -> only x,y
hq = ditch_small_terms(vpa(hq),1e-4);
hq = [1 0 0 ; 0 1 0]*hq;

%% Differentiate the constraint
%Constraint index 2: Hv = 0 (d/dt{h(q)} = d/dq{h(q)} * qt = jacobian*v
H  = jacobian(hq,qs); %Hv = 0, v = qt;

%Constraint index 1: H*vt + Ht*v = 0 
Ht = diff(H ,t); % H*vt + Ht*v

%subs 
Ht1 = [1 0]*Ht;
Ht1s = subs(Ht1,diff(qs,t),qts);

Ht2 = [0 1]*Ht;
Ht2s = subs(Ht2,diff(qs,t),qts);

Ht123 = [Ht1s;Ht2s];


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

hqs = subs(hq(t)   ,[qs;qts]',state_out');
Hs  = subs(H(t)    ,[qs;qts]',state_out');
Hts = subs(Ht123(t),[qs;qts]',state_out');






