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

plt.xdot_tool1(:, loop) = pandaArm.xdot.ArmL.tool;
plt.xdot_tool2(:, loop) = pandaArm.xdot.ArmR.tool;

end