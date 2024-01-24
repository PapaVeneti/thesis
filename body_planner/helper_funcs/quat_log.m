function logq = quat_log(q)
%quat_log: This functions returns the quaternion logarithm accordin to
%this: https://web.archive.org/web/20170705123142/http://www.lce.hut.fi/~ssarkka/pub/quat.pdf
%
%q = [w q1 q2 q3]

w  = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
v = [q(2);q(3);q(4)];

%normally = 1
normq = sqrt( w^2 + q1^2 + q2^2 + q3^2);


logq = [    log(normq);...
            v/norm(v) * acos(w/normq) ];

% acos \in [-pi/2, pi/2]



end