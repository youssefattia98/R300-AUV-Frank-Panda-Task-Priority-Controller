function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

        switch mission.phase
            case 1  % REACH GOAL FRAME
                %pandaArm.Aa.tool = eye(6);
                pandaArm.A.A1 = 1;

            case 2 % GRASP AND MOVE THE OBJECT
            case 3 % STOP any motion 
                pandaArm.Aa.tool = zeros(6);
        end

% Control tasks (equality)
%pandaArm.A.tool = eye(6)*pandaArm.Aa.tool;

% Reaching Goal (equality)
pandaArm.A.rg = eye(12) .* pandaArm.A.A1;

% Joint Limits Task
    
% Activation function: two combined sigmoids, which are at their maximum at the joint limits and approach zero between them    
% Safety Task (inequality)
% ... 

end
