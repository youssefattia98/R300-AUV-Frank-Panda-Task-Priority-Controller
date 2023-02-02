function [ ] = PrintPlot( plt, pandaArm )
Acrions_Transtion_time = plt.action_transition_time
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
hplot = plot(plt.t, plt.distLrg2);
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
hplot = plot(plt.t, plt.distRrg2);
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


%Left arm desired and actual linear and angular velocity
figure(4);
subplot(4,2,1)
hplot = plot(plt.t, plt.LDesLin);
title('LEFT ARM desired Linear Velocity [m/s]')
xlabel('time [s]');
ylabel('Linear Velocity [m/sec]');
set(hplot, 'LineWidth', 1);
legend('Vx','Vy','Vz');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,3)
hplot = plot(plt.t, plt.LActlin);
title('LEFT ARM actual Linear Velocity [m/s]')
xlabel('time [s]');
ylabel('Linear Velocity [m/sec]');
set(hplot, 'LineWidth', 1);
legend('Vx','Vy','Vz');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,5)
hplot = plot(plt.t, plt.LDesAng);
title('LEFT ARM desired Angular Velocity [rad/s]')
xlabel('time [s]');
ylabel('Angular Velocity [rad/sec]');
set(hplot, 'LineWidth', 1);
legend('Wx','Wy','Wz');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,7)
hplot = plot(plt.t, plt.LActAng);
title('LEFT ARM actual Angular Velocity [m/s]')
xlabel('time [s]');
ylabel('Angular Velocity [rad/sec]');
set(hplot, 'LineWidth', 1);
legend('Wx','Wy','Wz');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;

%right arm desired and actual linear and angular velocity

subplot(4,2,2)
hplot = plot(plt.t, plt.RDesLin);
title('Right ARM desired Linear Velocity [m/s]')
xlabel('time [s]');
ylabel('Linear Velocity [m/sec]');
set(hplot, 'LineWidth', 1);
legend('Vx','Vy','Vz');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,4)
hplot = plot(plt.t, plt.RActLin);
title('Righ ARM actual Linear Velocity [m/s]')
xlabel('time [s]');
ylabel('Linear Velocity [m/sec]');
set(hplot, 'LineWidth', 1);
legend('Vx','Vy','Vz');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,6)
hplot = plot(plt.t, plt.RDesAng);
title('Right ARM desired Angular Velocity [rad/s]')
xlabel('time [s]');
ylabel('Angular Velocity [rad/s]');
set(hplot, 'LineWidth', 1);
legend('Wx','Wy','Wz');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;
subplot(4,2,8)
hplot = plot(plt.t, plt.RActAng);
title('Right ARM actual Angular Velocity [m/s]')
xlabel('time [s]');
ylabel('Angular Velocity [rad/s]');
set(hplot, 'LineWidth', 1);
legend('Wx','Wy','Wz');
hold on;
for i = 1:3
    vline(plt.action_transition_time(i),'r');
end
hold off;

end

