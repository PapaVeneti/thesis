
%% Geodesic distance. Fix nan value if q = [1 0 0 0] -> zero error. 
x = linspace(0.2,1);

y1 = acos(x)./(sqrt(1-x.^2)+1e-9); %%This fixes the nan value, but it goes strait to zero
y2 = acos(x)./sqrt(1-x.^2);

figure(1)
plot(x,y1,'k')
hold on
plot(x,y2,'b')

%% example:
% initial state:
q0 = [1; 0; 0;0]; 
w0 = [0;0;0];
x0 = [q0;w0];

% reference:
eul_ref = [pi/2,0,0];
qref = eul2quat(eul_ref,'ZYX')';
wref = [0;0;0];
xref = [qref;wref];

figure(Name='Transforms')
hold on
plotTransforms([0,0,0],[1,0,0,0],"FrameAxisLabels","on","FrameSize",2,"FrameLabel","Inertial")
plotTransforms([0,0,0],qref,"FrameAxisLabels","on","FrameSize",2,"FrameLabel","Rotated")
hold off




error1 = quat_mul( qref',quat_inv(q0'));
error2 = quaternion_difference_matrix(qref)*q0';