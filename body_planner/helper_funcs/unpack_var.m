function x_unpacked = unpack_var(x,N)
%unpack_var: This functions unpacks variables from acados that are stacked
%
arguments
    x(1,:)
    N {mustBePositive}
end

len = length(x);
x_unpacked = zeros(N,len/N);

for i=1:N
    x_unpacked(i,:) = x(i:N:end); 
end

end