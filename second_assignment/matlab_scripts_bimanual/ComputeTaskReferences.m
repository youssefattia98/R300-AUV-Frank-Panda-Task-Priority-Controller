function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin

switch mission.phase
    case 1
        % Compute the tool position and orientation task reference
    
    case 2
        % Performe the rigid grasp of the object and move ite
    case 3
        % Stop any motions
end


