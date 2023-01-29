function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin

switch mission.phase
    case 1
        % Compute the distance and the misalignment between each arm end
        % effector and its goal position: ( RG reaching goal )
        [w_rho_Leeg , w_dist_Leeg] = CartError(pandaArm.ArmL.wTg, pandaArm.ArmL.wTt);
        pandaArm.xdot.ArmL.rg = [0.2 * (w_rho_Leeg); 0.2 * (w_dist_Leeg)];
        
        [w_rho_Reeg , w_dist_Reeg] = CartError(pandaArm.ArmR.wTg, pandaArm.ArmR.wTt);
        pandaArm.xdot.ArmR.rg = [0.2 * (w_rho_Reeg); 0.2 * (w_dist_Reeg)];
        
        % Sum up al together:
        pandaArm.xdot.rg = [pandaArm.xdot.ArmL.rg; pandaArm.xdot.ArmR.rg];
        
    
    case 2
        % Performe the rigid grasp of the object and move ite
    case 3
        % Stop any motions
end


