robot = importrobot('urdf/ntnu_legBODY.urdf')
show(robot,Visuals="off",Collisions="on")
robot.DataFormat = "column";

% config = homeConfiguration(robot);
% config(1).JointPosition = -1.31;
% config(2).JointPosition = -1;
% show(robot,config)
% 
% config(4)

%% Limits:
qmh_lim = [-2;0.5];
q11_lim = [-1.31;0];
q21_lim = [-1.323;0.5];

%Discretization:
Nmh = 12;
N11 = 40;
N21 = 40;

QMH = linspace(qmh_lim(1),qmh_lim(2),Nmh);
Q11 = linspace(q11_lim(1),q11_lim(2),N11);
Q21 = linspace(q21_lim(1),q21_lim(2),N21);


[Qmesh11,Qmesh21]  = meshgrid(Q11,Q21);

A = zeros(Nmh,N21,N11);

imh = 1;
for qmh=QMH
    i11 = 1;
    for q11=Q11
        i21 = 1;
        for q21=Q21
            q = [qmh;q11;q21];
            A(imh,i21,i11) = robot.checkCollision(q,SkippedSelfCollisions="parent");
        i21 = i21 +1;
        end
        i11 = i11+1;
    end
    imh = imh+1
end

%% my constraints:
q11_c = [-0.5 ,  -0.6];
q21_c = [-1.24, -0.71];

q11_c = [-0.48 ,  -0.6];
q21_c = [-1.2, -0.71];

q11_c = [ -0.8430 -1.3337];
q21_c = [-0.2960 0.5753];


coef = polyfit(q11_c,q21_c,1);
plot_c1 = q11_lim*coef(1) + coef(2);


%% constraint 2 -> Very good constraint -> Avoid hitting the body
q11_2 = [-0.83, -0.61];
qmh_2 = [-1.55, -2];


coef = polyfit(qmh_2,q11_2,1);
con2  = QMH* coef(1) + coef(2);

%% circle constr

xc_c1 = [-5.2056;-2.7445];
xc_c1 = [-5.20;-2.75];

r_c1 = 4.9452;
r_c1 = 5;

%% plots
figure(Name="angle_space");
set(gcf, 'Position', get(0, 'Screensize'));
xlabel('q11',FontSize=30);
ylabel('q21',FontSize=30);
for i=Nmh:-1:1
i
QMH(i)
title_txt = ['Q_{MH} = ', num2str(QMH(i),3)];

s1 = surf(Qmesh11,Qmesh21,reshape(A(i,:,:),N21,N11),EdgeColor="interp",FaceColor="interp");
hold on
plot3(q11_lim,plot_c1,[1 1],Color='r',LineWidth=2,LineStyle='--')
hold on
plot3(con2(i)*[1,1],q21_lim,[1 1],Color='r',LineWidth=2)
hold on
plot_circle_in_z(xc_c1,r_c1,1,50)
hold off
title(title_txt);
xlabel('q11',FontSize=30);
ylabel('q21',FontSize=30);
ylim(q21_lim + [-0.1; 0.1])
xlim(q11_lim + [-0.1; 0.1])
colorbar('Ticks',[0,1],...
         'TickLabels',{'No collision', 'Collision'})
view([0,0,1])
pause(1)


end
