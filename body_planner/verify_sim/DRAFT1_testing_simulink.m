%% add simulink block to path:
sys_path = path; 
helper_dir = erase(pwd,"verify_sim");
if strfind(sys_path,helper_dir)
    disp('quadruped is alread added to path');
else
    addpath(helper_dir) %for current session
end
clear sys_path  helper_dir;
%% system parameter:
I = zeros(3);
I(1,2) =-0.00085624;
I(1,3) = 0.0025397;
I(2,3) = 0.00030749;

I = I+I';

I(1,1) = 0.4897949;
I(2,2) = 0.92002036;
I(3,3) = 1.00669705;

%% init:
tau = [1;1;1];

%orientation
q0 = [1;0;0;0];
% velocity:
w0 = [0;0;0];
%initial state
y0 = [q0;w0];

%% Simulation comparison:
sim_time= [0,2];

opts  = odeset('RelTol',1e-6,'AbsTol',1e-9);
fun  = @(t,y) euler_RB([],y,I,tau);
[t,y]=ode45(fun,sim_time ,y0 ,opts);

% q0 must be defined
out = sim('body_simulation.slx',sim_time);

%% Plot orientation (detailed)
simTime = out.orientation.Time;
simData = [out.orientation.Data;out.omega.Data];
simData =  reshape(simData, 7,length(simTime) );

figure(Name='Orientation Detailed')
States = { '$w$','$q_1$','$q_2$','$q_3$','$\omega_1$','$\omega_2$','$\omega_3$',};
for i=1:length(States)
    subplot(length(States), 1, i);

    simu_txt = ['simulink:',States{i}];
    plot(simTime,simData(i,:),'k-',DisplayName=simu_txt)
    hold on;

    sim_txt = ['custom_sim:',States{i}];
    plot(t,y(:,i),'b--',DisplayName=sim_txt);


    grid on; 
    hold on;
    ylabel(States{i},'interpreter','latex',FontSize=25);
    if i < 5
        ylim([-1.1 1.1]);
    end
    legend()
end
xlabel('t [s]')


%% Frame animation
% plot( out.omega )
% plot( out.orientation)


tsimu = out.orientation.Time; 
N  = size(tsimu);
data= out.orientation.Data;


animation_fig = figure("Name",'Frame Animation');
plotTransforms([0,0,0],[1,0,0,0],"FrameAxisLabels","on","FrameSize",2,"FrameLabel","Inertial")
hold on
for i=1:N-1
    figure(animation_fig);
    plotTransforms([0,0,0],data(:,1,i)',"FrameAxisLabels","on","FrameSize",2,"FrameLabel","Body")
    pause( 0.5*( tsimu(2)-tsimu(1) ))
end
figure(animation_fig);
plotTransforms([0,0,0],data(:,1,i+1)',"FrameAxisLabels","on","FrameSize",2,"FrameLabel","Body")
hold off