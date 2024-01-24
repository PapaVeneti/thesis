function qq = quat_mul(q1,q2)
%quat_mul: A function to multiply quaternions.
%
%quaternions are in the form: [w,q1,q2,q3]
%used: https://personal.utdallas.edu/~sxb027100/dock/quaternion.html


qq = zeros(4,1);
w1 = q1(1); 
v1 = q1(2:4);

w2 = q2(1);
v2 = q2(2:4);

% operation:
qq(1)     = w1*w2 - (   v1(1)*v2(1) +...
                        v1(2)*v2(2) +...
                        v1(3)*v2(3));

qq(2:4,1) = w1*v2 + w2*v1 + exterior(v1)* v2;


end