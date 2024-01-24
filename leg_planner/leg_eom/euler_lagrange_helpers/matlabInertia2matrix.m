function I  = matlabInertia2matrix(Im,CoM,m)
%matlabInertia2matrix: This function gets the inertia returned from a
%rigidbody from matlab: `[Ixx Iyy Izz Iyz Ixz Ixy]` and returns a
%matrix of that inertia.
arguments
    Im (1,6) {mustBeNumeric}
    CoM (1,3) {mustBeNumeric}
    m (1,1) {mustBePositive}
end

I = [Im(1) Im(6) Im(5); Im(6) Im(2) Im(4); Im(5) Im(4) Im(3)];
I = I + m*exterior(CoM)^2;

end