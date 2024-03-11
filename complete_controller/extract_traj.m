function y = extract_traj(tspan,x,nstates)

len_x  = length(x);
n = len_x/nstates;

y = zeros(n,nstates+1);
y(:,1) = linspace(tspan(1),tspan(2),n);

for i =1:nstates

y(:,i+1) = x(i:nstates:len_x);
end