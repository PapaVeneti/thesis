syms wd q1d q2d q3d

%% Quaternion product (for error):
Z = sym('s',4);


Z(2:4,2:4) = wd*eye(3) + exterior([q1d;q2d;q3d]);
Z(2:4,1) = [q1d;q2d;q3d];
Z(1,:) = [wd -q1d -q2d -q3d];

% quat product:

QD = Z*diag([1,-1,-1,-1]);

%% Quaternion error:
syms w q1 q2 q3

qe = QD*[w;q1;q2;q3];

%% Quaternion log: (+temporary vars)
ve = qe(2:4);
normv = sqrt(ve(1)^2+ve(2)^2+ve(3)^2);
simplify(normv)

log2 = acos(ve(1))/(normv+1e-9)*ve;

logq = [1;log2];


