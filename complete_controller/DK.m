function q_est = DK(q)
%% defines
qMH_offset = -pi/2;
q11_offset = 2.3095;
q21_offset = 1.3265;
q12_offset = 0.83482;
q22_offset = -1.3233;

j11_Dx = 0.046; %Horizontal distance of joint11 from MH frame
l11    = 0.18;
l21    = 0.29977;

j12_Dx = 0.136; %Horizontal distance of joint12 from MH frame
l12    = 0.18;
l22    = 0.29929;

j_Dy = -1.1338e-05; %Vertical Distance of joint11 and joint12 from MH frame
pj11 = [0.046, -1.1338e-05]; % `pj11` is the center of joint11
pj12 = [0.136, -1.1338e-05]; % `pj12` is the center of joint12 
%% Intersection point
q11 = -q(2) + q11_offset;
q12 = -q(4) + q12_offset;

%centers
p1c(1) = j11_Dx + l11*cos(q11);
p1c(2) = j_Dy   + l11*sin(q11);

p2c(1) = j12_Dx + l12*cos(q12);
p2c(2) = j_Dy   + l12*sin(q12);

% normal and center vector
vc = p2c-p1c;
d  = norm(vc); %distance of circle centers `p1c` and `p2c`
vc = vc/d;
vn = [-vc(2),vc(1)];

%%triangle solution
a = ( (d*d) + (l21*l21) - (l22*l22) )/( 2*d );
h = sqrt( (l21*l21) - (a*a));

p_EE_22d = p1c + (a*vc) + (h*vn);


%% joint angles
link11_v = p1c-pj11;
link12_v = p2c-pj12;

link21_v = p_EE_22d-p1c;
link22_v = p_EE_22d-p2c;

theta1 = acos(dot(link11_v, link21_v) / (norm(link11_v) * norm(link21_v)));
theta2 = -acos(dot(link12_v, link22_v) / (norm(link12_v) * norm(link22_v)));

% Check direction
link11_vn = link11_v / norm(link11_v);
side1_extended_x = link11_vn(1) * (l11 + l21);

link12_vn = link12_v / norm(link12_v);
side2_extended_x = link12_vn(1) * (l12 + l22);

if p_EE_22d(1) < side1_extended_x
    % joint12 is outwards
    theta1 = -theta1;
elseif p_EE_22d(1) > side2_extended_x
    % joint22 is outwards
    theta2 = -theta2;
end

q_est    = zeros(5,1);
q_est(1) = q(1);
q_est(2) = q(2);
q_est(3) = theta1 - q21_offset;
q_est(4) = q(4);
q_est(5) = theta2 - q22_offset;

end