function QDM = quaternion_difference_matrix(qd)
%quaternion_difference_matrix: A function to return the difference matrix
%defined here: https://drum.lib.umd.edu/bitstreams/1bf4826a-1b76-4bc6-9a3b-07a67136eff7/download
%
%quaternions are in the form: [w,q1,q2,q3]


w  = qd(1);
q1 = qd(2);
q2 = qd(3);
q3 = qd(4);


QDM = zeros(4);


QDM(1,:) = [  w, q1, q2, q3];
QDM(2,:) = [-q1,  w, q3,-q2];
QDM(3,:) = [-q2,-q3,  w, q1];
QDM(4,:) = [-q3, q2,-q1,  w];


% dq = qref (x) q0' , where (x) is the quaternion product
% q0 = diag([1,-1,-1,-1])*q0 = D*q0
% qref (x) = Ksi matrix from book
% Ksi*D  = QDM
%
QDM(1,:) = [ w,  q1,  q2,  q3];
QDM(2,:) = [q1,  -w,  q3, -q2];
QDM(3,:) = [q2, -q3,  -w,  q1];
QDM(4,:) = [q3,  q2, -q1,  -w];

end