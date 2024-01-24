function [state_dot] = euler_RB(~,state,I,tau)
%euler_RB: This functions simulates the euler rigid body dynamics of a body
%given the current `state`, the inertia about in the body frame `I` and the
%torque that is exerted on the body frame `tau`. 

%q = [w,q1,q2,q3] %matlab convention

q = state(1:4,1);
w = state(5:7,1);

%1: Quaternion derivatives
% for q= [q1,q2,q3,w]
wquat = [
    0,-w(1),-w(2),-w(3);
 w(1),    0, w(3),-w(2);
 w(2),-w(3),    0, w(1);
 w(3), w(2),-w(1),    0;
]; 

% wquat = [
%     0, w(3), w(2),-w(1);
% -w(3),    0,-w(1),-w(2);
% -w(2), w(1),    0, w(3);
%  w(1), w(2),-w(3),    0;
% ];

%2: exterior product of omega
w_ext = [
0 -w(3) w(2);
w(3) 0 -w(1);
-w(2) w(1) 0];


%% Derivatives
qt = 1/2*wquat*q;
wt = I\ ( tau - w_ext*(I*w) ) ;

state_dot = [qt;wt];


end