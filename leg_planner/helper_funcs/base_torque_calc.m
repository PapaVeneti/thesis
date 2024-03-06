function tmh = base_torque_calc(qmh,t)

%[t1; -sin(q1)*(t11+t21) ; cos(q1)*(t11+t21)]


tmh = t;

% tmh(:,1) = t(:,1);
tmh(:,2) = -sin(qmh).*(- ( t(:,2)+t(:,3) ) );
tmh(:,3) =  cos(qmh).*(- ( t(:,2)+t(:,3) ) );