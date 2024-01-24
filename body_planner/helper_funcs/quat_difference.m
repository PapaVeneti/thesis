function dq = quat_difference(qref,q)
%quat_difference: Returns attitude error in quaternions:
%
%qref = dq* q -> dq = qref * inv(q)
%
%q = [w q1 q2 q3]; 
arguments
    qref(4,1)
    q(4,1)
end


dq = quat_mul( qref,quat_inv(q));

end