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
plt.rhoLrg1(:, loop) = norm(pandaArm.ArmL.w_rho_Leeg1);
plt.rhoRrg1(:, loop) = norm(pandaArm.ArmR.w_rho_Reeg1);
plt.distLrg1(:, loop) = norm(pandaArm.ArmL.w_dist_Leeg1);
plt.distRrg1(:, loop) = norm(pandaArm.ArmR.w_dist_Reeg1);
plt.ajlL(:, loop) = diag(pandaArm.ArmL.A.jl);
plt.ajlR(:, loop) = diag(pandaArm.ArmR.A.jl);
% Cooperative velocities
plt.LCoopAng(:, loop) = pandaArm.ArmL.x_dot.coop(1:3,:);
plt.LCoopLin(:, loop) = pandaArm.ArmL.x_dot.coop(4:6,:);
plt.RCoopAng(:, loop) = pandaArm.ArmR.x_dot.coop(1:3,:);
plt.RCoopLin(:, loop) = pandaArm.ArmR.x_dot.coop(4:6,:) ;
% Non-Cooperative velocities
plt.LNonCoopAng(:, loop) = pandaArm.ArmL.x_dot.mu(1:3,:);
plt.LNonCoopLin(:, loop) = pandaArm.ArmL.x_dot.mu(4:6,:);
plt.RNonCoopAng(:, loop) = pandaArm.ArmR.x_dot.mu(1:3,:);
plt.RNonCoopLin(:, loop) = pandaArm.ArmR.x_dot.mu(4:6,:);

plt.action_transition_time =pandaArm.ta;


end