function [] = plot_circle_in_z(xc,r,z,N)

t = linspace(0,2*pi,N);
x = xc(1) + r*cos(t);
y = xc(2) + r*sin(t);


plot3(x,y,z*ones(N,1),Color='r',LineWidth=2);