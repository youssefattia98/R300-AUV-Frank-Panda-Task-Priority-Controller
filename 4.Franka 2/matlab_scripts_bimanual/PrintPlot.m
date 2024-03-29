function [ ] = PrintPlot( plt, pandaArm )

% some predefined plots
% you can add your own
x = plt.action_transition_time

figure(1);
subplot(2,1,1);
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
subplot(2,1,2);
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

figure(2);
subplot(2,1,1);
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
subplot(2,1,2);
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

figure(3);
subplot(2,2,1)
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
subplot(2,2,2)
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
subplot(2,2,3)
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
subplot(2,2,4)
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




figure(4);
subplot(2,1,1)
hplot = plot(plt.t, plt.rhoLrg1);
title('LEFT ARM missalignment to second goal [rad]')
xlabel('time [s]');
ylabel('misalignment [rad]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(2,1,2)
hplot = plot(plt.t, plt.rhoRrg1);
title('RIGHT ARM missalignment to second goal [rad]')
xlabel('time [s]');
ylabel('misalignment [rad]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;

figure(5);
subplot(2,1,1)
hplot = plot(plt.t, plt.w_dist_Leeg2);
title('LEFT ARM distance to third goal [m]')
xlabel('time [s]');
ylabel('distance [m]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(2,1,2)
hplot = plot(plt.t, plt.w_dist_Reeg2);
title('RIGHT ARM distance to third goal [m]')
xlabel('time [s]');
ylabel('distance [m]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;

figure(6);
subplot(2,1,1)
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
subplot(2,1,2)
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

figure(7);
subplot(4,1,1)
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
subplot(4,1,2)
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
subplot(4,1,3)
hplot = plot(plt.t, plt.w_dist_Leeg2);
title('LEFT ARM distance to third goal [m]')
xlabel('time [s]');
ylabel('distance [m]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,1,4)
hplot = plot(plt.t, plt.w_dist_Reeg2);
title('RIGHT ARM distance to third goal [m]')
xlabel('time [s]');
ylabel('distance [m]');
set(hplot, 'LineWidth', 1);
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;

% Cooperative velocities for the right and left arms
figure(8);
subplot(4,2,1)
hplot = plot(plt.t, plt.LCoopAng);
title('LEFT ARM angular velocity in cooperative hierarchy[rad/s]')
xlabel('time [s]');
ylabel('[rad/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x','omega_y','omega_z');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,2)
hplot = plot(plt.t, plt.LCoopLin);
title('LEFT ARM linear velocity in cooperative hierarchy[m/s]')
xlabel('time [s]');
ylabel('[m/s]');
set(hplot, 'LineWidth', 1);
legend('V_x','V_y','V_z');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,3)
hplot = plot(plt.t, plt.LNonCoopAng);
title('LEFT ARM angular velocity in Non-cooperative hierarchy[rad/s]')
xlabel('time [s]');
ylabel('[rad/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x','omega_y','omega_z');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,4)
hplot = plot(plt.t, plt.LNonCoopLin);
title('LEFT ARM linear velocity in Non-cooperative hierarchy[m/s]')
xlabel('time [s]');
ylabel('[m/s]');
set(hplot, 'LineWidth', 1);
legend('V_x','V_y','V_z');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
% Non-Cooperative velocities for the right and left arms
subplot(4,2,5)
hplot = plot(plt.t, plt.RCoopAng);
title('RIGHT ARM angular velocity in cooperative hierarchy[rad/s]')
xlabel('time [s]');
ylabel('[rad/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x','omega_y','omega_z');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,6)
hplot = plot(plt.t, plt.RCoopLin);
title('RIGHT ARM linear velocity in cooperative hierarchy[m/s]')
xlabel('time [s]');
ylabel('[m/s]');
set(hplot, 'LineWidth', 1);
legend('V_x','V_y','V_z');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,7)
hplot = plot(plt.t, plt.RNonCoopAng);
title('RIGHT ARM angular velocity in Non-cooperative hierarchy[rad/s]')
xlabel('time [s]');
ylabel('[rad/s]');
set(hplot, 'LineWidth', 1);
legend('omega_x','omega_y','omega_z');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,8)
hplot = plot(plt.t, plt.RNonCoopLin);
title('RIGHT ARM linear velocity in Non-cooperative hierarchy[m/s]')
xlabel('time [s]');
ylabel('[m/s]');
set(hplot, 'LineWidth', 1);
legend('V_x','V_y','V_z');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
end

