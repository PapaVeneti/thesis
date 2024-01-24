function log_er = geodesic_distance_SX(qd,q)
wd   = qd(1); w  = q(1);
q1d  = qd(2); q1 = q(2);
q2d  = qd(3); q2 = q(3);
q3d  = qd(4); q3 = q(4);




log_er = ...
    [    1
 (acos(q2*q3d - q3*q2d + q1d*w - q1*wd)*(q2*q3d - q3*q2d + q1d*w - q1*wd))/(((q1*q2d - q2*q1d + q3d*w - q3*wd)^2 + (q1*q3d - q3*q1d - q2d*w + q2*wd)^2 + (q2*q3d - q3*q2d + q1d*w - q1*wd)^2)^(1/2) + 1/1000000000)
-(acos(q2*q3d - q3*q2d + q1d*w - q1*wd)*(q1*q3d - q3*q1d - q2d*w + q2*wd))/(((q1*q2d - q2*q1d + q3d*w - q3*wd)^2 + (q1*q3d - q3*q1d - q2d*w + q2*wd)^2 + (q2*q3d - q3*q2d + q1d*w - q1*wd)^2)^(1/2) + 1/1000000000)
 (acos(q2*q3d - q3*q2d + q1d*w - q1*wd)*(q1*q2d - q2*q1d + q3d*w - q3*wd))/(((q1*q2d - q2*q1d + q3d*w - q3*wd)^2 + (q1*q3d - q3*q1d - q2d*w + q2*wd)^2 + (q2*q3d - q3*q2d + q1d*w - q1*wd)^2)^(1/2) + 1/1000000000)];