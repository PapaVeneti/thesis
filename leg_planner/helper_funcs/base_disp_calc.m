function displacement = base_disp_calc(N,DT,qmh,tau)

%body frames
iyy=0.92002036;
ixx= 0.4897949;
izz= 1.00669705;

%in leg frame 
I = diag([ixx,izz,iyy]);


%torque body -> legs
tmh = base_torque_calc ( qmh,tau);

%displacement
A = zeros(1,N);

acc =  tmh*I;

for i =1:N
    for j=1:i
        A(j) = (i-j+0.5)*DT^2;
    end
    displacement(i,1:3) = A*acc;
end




