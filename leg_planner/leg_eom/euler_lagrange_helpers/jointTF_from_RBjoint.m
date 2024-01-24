function Tf = jointTF_from_RBjoint(joint,var)
%jointTF_from_RBjoint: This function returns a symbolic transformation
%matrix that represents the transformation from a random position of a
%joint to the home position (generalized pos = 0).
%
%Inputs:
%   1. joint: matlab `rigidBodyJoint`
%   2. var:   symbolic that represents the generalized postion
%
%with the convention T_AB = transformation of {B} to {A}, it returns:
%    T_j0jq 
%where j0 is the home frame for q=0.  
%
%Notes: Currently it supports only prismatic and revolute joints
%% Type checking:

if ~matches(class(joint),'rigidBodyJoint')
    error('Input 1 must be of type: "rigidBodyJoint"');

elseif ~matches(class(var),'symfun')
    error('Input 2 must be of type: "symfun"');
    
end

%% Transformation:
switch joint.Type

    case 'prismatic'
        p = joint.JointAxis*var; %joint axis = 1x3.
        Tf = [eye(3) , p'; zeros(1,3),1];

    case 'revolute'
        %Using rodriguez formula for rotations to handle general axis
        %https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

        K  = exterior(joint.JointAxis);
        R  = eye(3) + sin(var)*K + (1-cos(var))*K^2; 
        Tf = [R , zeros(3,1); zeros(1,3),1];

    otherwise 
        disp('Currently only supports: ["prismatic","revolute"] joints.');
        error('Unsupported joint type.')
end


end
