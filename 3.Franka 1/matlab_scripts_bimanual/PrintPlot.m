function [ ] = PrintPlot( plt, pandaArm )
x = plt.action_transition_time
%left & right arm configuration + left & right arm joint velocities
figure(1);
subplot(4,1,1);
hplot = plot(plt.t, plt.q);
title('LEFT ARM configuration [rad]')
xlabel('time [s]');
ylabel('configuration angle [rad]');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,2);
hplot = plot(plt.t, plt.q2);
title('RIGHT ARM configuration [rad]')
xlabel('time [s]');
ylabel('configuration angle [rad]');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,3);
hplot = plot(plt.t, plt.q_dot);
title('LEFT ARM joint velocities [rad/s]')
xlabel('time [s]');
ylabel('configuration speed [rad/s]');
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,4);
hplot = plot(plt.t, plt.q_dot2);
title('RIGHT ARM joint velocities [rad/s]')
xlabel('time [s]');
ylabel('configuration speed [rad/s]');
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;

%L&R distance to first goal + L&R missalignment to first goal
figure(2);
subplot(4,1,1)
hplot = plot(plt.t, plt.distLrg);
title('LEFT ARM distance to first goal [m]')
xlabel('time [s]');
ylabel('distance [m]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,2)
hplot = plot(plt.t, plt.distRrg);
title('RIGHT ARM distance to first goal [m]')
xlabel('time [s]');
ylabel('distance [m]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,3)
hplot = plot(plt.t, plt.rhoLrg);
title('LEFT ARM missalignment to first goal [rad]')
xlabel('time [s]');
ylabel('misalignment [rad]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,4)
hplot = plot(plt.t, plt.rhoRrg);
title('RIGHT ARM missalignment to first goal [rad]')
xlabel('time [s]');
ylabel('misalignment [rad]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;

%L&R distance to second goal + L&R Joint lmits tasks actvation
figure(3);
subplot(4,1,1)
hplot = plot(plt.t, plt.distLrg1);
title('LEFT ARM distance to second goal [m]')
xlabel('time [s]');
ylabel('distance [m]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,2)
hplot = plot(plt.t, plt.distRrg1);
title('RIGHT ARM distance to second goal [m]')
xlabel('time [s]');
ylabel('distance [m]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,3)
hplot = plot(plt.t, plt.ajlL);
title('LEFT ARM joint limit activation function [-]')
xlabel('time [s]');
ylabel('[-]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,4)
hplot = plot(plt.t, plt.ajlR);
title('RIGHT ARM joint limit activation function [-]')
xlabel('time [s]');
ylabel('[-]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;

end

