function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin

% Reaching goal task (orientation and position together)
[w_rho_Leeg , w_dist_Leeg] = CartError(pandaArm.ArmL.wTg, pandaArm.ArmL.wTt);
pandaArm.xdot.ArmL.rg = [0.2 * (w_rho_Leeg); 0.2 * (w_dist_Leeg)];
pandaArm.ArmL.w_rho_Leeg = w_rho_Leeg;
pandaArm.ArmL.w_dist_Leeg = w_dist_Leeg;

[w_rho_Reeg , w_dist_Reeg] = CartError(pandaArm.ArmR.wTg, pandaArm.ArmR.wTt);
pandaArm.xdot.ArmR.rg = [0.2 * (w_rho_Reeg); 0.2 * (w_dist_Reeg)];
pandaArm.ArmR.w_rho_Reeg = w_rho_Reeg;
pandaArm.ArmR.w_dist_Reeg = w_dist_Reeg;


% Joint limit task:
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
for i = 1 : size(jlmin, 1)
    mean(i) = (jlmin(i) + jlmax(i)) / 2;
    if pandaArm.ArmL.q(i) <= mean(i)
        pandaArm.xdot.ArmL.jl(i, 1) = 0.2 * (jlmin(i) - pandaArm.ArmL.q(i) + 0.1);
    else
        pandaArm.xdot.ArmL.jl(i, 1) = 0.2 * (jlmax(i) - pandaArm.ArmL.q(i) - 0.1);
    end
end

for i = 1 : size(jlmin, 1)
    mean(i) = (jlmin(i) + jlmax(i)) / 2;
    if pandaArm.ArmR.q(i) <= mean(i)
        pandaArm.xdot.ArmR.jl(i, 1) = 0.2 * (jlmin(i) - pandaArm.ArmR.q(i));
    else
        pandaArm.xdot.ArmR.jl(i, 1) = 0.2 * (jlmax(i) - pandaArm.ArmR.q(i));
    end
end


% Reaching goal1 task (orientation and position together)
[w_rho_Leeg1 , w_dist_Leeg1] = CartError(pandaArm.ArmL.wTg1, pandaArm.ArmL.wTo);
pandaArm.xdot.ArmL.rg1 = [0.2 * (w_rho_Leeg1); 0.2 * (w_dist_Leeg1)];
pandaArm.ArmL.w_dist_Leeg1 = w_dist_Leeg1;
pandaArm.ArmL.w_rho_Leeg1 = w_rho_Leeg1;

[w_rho_Reeg1 , w_dist_Reeg1] = CartError(pandaArm.ArmR.wTg1, pandaArm.ArmR.wTo);
pandaArm.xdot.ArmR.rg1 = [0.2 * (w_rho_Reeg1); 0.2 * (w_dist_Reeg1)];
pandaArm.ArmR.w_dist_Reeg1 = w_dist_Reeg1;
pandaArm.ArmR.w_rho_Reeg1 = w_rho_Reeg1;

% Reaching goal2 task (orientation and position together)
[w_rho_Leeg2 , w_dist_Leeg2] = CartError(pandaArm.ArmL.wTg2, pandaArm.ArmL.wTt);
pandaArm.xdot.ArmL.rg2 = [0; 0; 0; 0.2 * (w_dist_Leeg2)];
pandaArm.ArmL.w_dist_Leeg2 = w_dist_Leeg2;

[w_rho_Reeg2 , w_dist_Reeg2] = CartError(pandaArm.ArmR.wTg2, pandaArm.ArmR.wTt);
pandaArm.xdot.ArmR.rg2 = [0; 0; 0; 0.2 * (w_dist_Reeg2)];
pandaArm.ArmR.w_dist_Reeg2 = w_dist_Reeg2;

% Finish task
pandaArm.ArmL.xdot.f = zeros(6, 1);
pandaArm.ArmR.xdot.f = zeros(6, 1);
end



