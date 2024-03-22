kk    = load('../leg_planner/Trajectories/N150Th075_rl_TC_01.mat');
xinit = kk.xtraj;

kk    = load('../leg_planner/Trajectories/N150Th075_rl_TC_01_u.mat');
uinit = kk.utraj;


N = 150;
Th = 0.75;
t = linspace(0,Th,N+1)';
nx=10;nu=3;

x_ref = zeros(10*(N+1),1);
u_ref = zeros(3*N,1);


for i=0:N
    x_ref( i*nx+1  : (i+1)*nx,1   ) = xinit(:,i+1);
end

for i=0:N-1
    u_ref( i*nu+1  : (i+1)*nu,1   ) = uinit(:,i+1);
end

