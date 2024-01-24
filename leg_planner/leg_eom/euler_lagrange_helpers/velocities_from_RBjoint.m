function [w,u] = velocities_from_RBjoint(joint,var)
%velocities_from_RBjoint: This function returns the velocity [u;w] of a
%joint

switch joint.Type

    case 'prismatic'
        u = joint.JointAxis' * var; 
        w = [0;0;0];

    case 'revolute'
        w = joint.JointAxis' * var; 
        u = [0;0;0];

end