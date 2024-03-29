function [ plt ] = UpdateDataPlot( plt, pandaArm, t, loop, mission )

% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script


plt.t(:, loop) = t;
plt.q(:, loop) = pandaArm.ArmL.q;
plt.q_dot(:, loop) = pandaArm.ArmL.q_dot;
plt.q2(:, loop) = pandaArm.ArmR.q;
plt.q_dot2(:, loop) = pandaArm.ArmR.q_dot;

plt.rhoLrg(:, loop) = norm(pandaArm.ArmL.w_rho_Leeg);
plt.rhoRrg(:, loop) = norm(pandaArm.ArmR.w_rho_Reeg);
plt.distLrg(:, loop) = norm(pandaArm.ArmL.w_dist_Leeg);
plt.distRrg(:, loop) = norm(pandaArm.ArmR.w_dist_Reeg);
plt.distLrg1(:, loop) = norm(pandaArm.ArmL.w_dist_Leeg1);
plt.distRrg1(:, loop) = norm(pandaArm.ArmR.w_dist_Reeg1);
plt.distLrg2(:, loop) = norm(pandaArm.ArmL.w_dist_Leeg2);
plt.distRrg2(:, loop) = norm(pandaArm.ArmR.w_dist_Reeg2);
plt.ajlL(:, loop) = diag(pandaArm.A.jl(1:7, 1:7));
plt.ajlR(:, loop) = diag(pandaArm.A.jl(8:14, 8:14));
plt.action_transition_time =pandaArm.ta;

if mission.phase == 2
    plt.RDesAng(:, loop) = [0 0 0]';
    plt.RDesLin(:, loop) = pandaArm.xdot.ArmR.rg1(4:6, :)';
    plt.LDesAng(:, loop) = [0 0 0]';
    plt.LDesLin(:, loop) = pandaArm.xdot.ArmL.rg1(4:6, :)';
elseif mission.phase == 3
    plt.RDesAng(:, loop) = [0 0 0]';
    plt.RDesLin(:, loop) = pandaArm.xdot.ArmR.rg2(4:6, :)';
    plt.LDesAng(:, loop) = [0 0 0]';
    plt.LDesLin(:, loop) = pandaArm.xdot.ArmL.rg2(4:6, :)';
else
    plt.RDesAng(:, loop) = [0 0 0]';
    plt.RDesLin(:, loop) = [0 0 0]';
    plt.LDesAng(:, loop) = [0 0 0]';
    plt.LDesLin(:, loop) = [0 0 0]';
end
plt.RActAng(:, loop) = pandaArm.xdot.ArmL.actual(1:3, :);
plt.RActLin(:, loop) = pandaArm.xdot.ArmR.actual(4:6, :);
plt.LActAng(:, loop) = pandaArm.xdot.ArmL.actual(1:3, :);
plt.LActlin(:, loop) = pandaArm.xdot.ArmL.actual(4:6, :);

end