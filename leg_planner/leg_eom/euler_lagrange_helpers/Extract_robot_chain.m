function robot_chain = Extract_robot_chain(robot)
%Extract_robot_chain: 
%Get the chain that leads to the base link for each link in the kinematic 
%chain.
%
%INPUTS:
% 1. robot: matlab rigid body tree object



num_bodies = robot.NumBodies;
robot_chain = cell(num_bodies,1);

base = robot.Base;

for ib= 1:num_bodies

    robot_chain{ib} = ib;
    curr_body = robot.Bodies{ib};

    in = ib-1; %index of parent. 

    %check if `parent is of current body` is `base` (end_condition)
    while ~matches( curr_body.Parent.Name,base.Name)
        %select the parent 
        curr_body =  curr_body.Parent;
        
        %get index of the parent
        while ~matches(robot.BodyNames{in},curr_body.Name)
            in = in-1;
        end

        %add it to the chain map
        robot_chain{ib} = horzcat(robot_chain{ib},in);
    end %finding parent loop
end %body loop