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

plt.rhoLrg(:, loop) = norm(w_rho_Leeg);
plt.rhoRrg(:, loop) = norm(w_rho_Reeg);
plt.distLrg(:, loop) = norm(w_dist_Leeg);
plt.distRrg(:, loop) = norm(w_dist_Reeg);
plt.distLrg1(:, loop) = norm(pandaArm.ArmL.w_dist_Leeg1);
plt.distRrg1(:, loop) = norm(pandaArm.ArmR.w_dist_Reeg1);
plt.ajlL(:, loop) = diag(pandaArm.A.jl(1:7, 1:7));
plt.ajlR(:, loop) = diag(pandaArm.A.jl(7:14, 7:14));

end