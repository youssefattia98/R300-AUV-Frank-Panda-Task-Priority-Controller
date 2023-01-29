function [ ] = PrintPlot( plt, pandaArm )

% some predefined plots
% you can add your own

figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
title('LEFT ARM')
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
figure(2);
subplot(2,1,1);
hplot = plot(plt.t, plt.q2);
title('RIGHT ARM')
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot2);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
figure(3);
subplot(2,1,1)
hplot = plot(plt.t, plt.xdot_tool1);
title('LEFT ARM - EE Cartestian Velocities')
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');
subplot(2,1,2)
hplot = plot(plt.t, plt.xdot_tool2);
title('RIGHT ARM - EE Cartestian Velocities')
set(hplot, 'LineWidth', 1);
legend('omega_x', 'omega_y', 'omega_z','xdot', 'ydot', 'zdot');

end

