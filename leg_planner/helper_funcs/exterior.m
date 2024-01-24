function vx = exterior(v)
%exterior: a function to compute the exterior product matrix of v

if class(v) == "symfun"
    x = [1 0 0]*v;
    y = [0 1 0]*v;
    z = [0 0 1]*v;

    vx = [
    0 -z y;
    z 0 -x;
    -y x 0];
else
    vx = [
    0 -v(3) v(2);
    v(3) 0 -v(1);
    -v(2) v(1) 0];

end



end