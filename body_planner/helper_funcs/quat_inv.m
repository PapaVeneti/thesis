function qinv = quat_inv(q)
%quat_inv(q): returns inverse quaternion so that q*q_inv = [1 0 0 0] = 1
%
%q = [w q1 q2 q3]

qinv = quat_conj(q)/ (norm(q)^2) ; 

end