function q = quat_conj(q_in)
%quat_conj: Returns conjugate of quaternion
%
%q = [w q1 q2 q3]

q = q_in;
q(2:4) = -q_in(2:4);

end