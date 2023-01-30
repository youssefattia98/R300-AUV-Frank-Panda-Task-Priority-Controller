function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
title('configuration [rad]');
xlabel('time [s]');
ylabel('configuration [rad]');
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
title('configuration speed [rad/s]');
xlabel('time [s]');
ylabel('configuration speed [rad/s]');
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');


figure(2);
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
title('vehicle position [m]');
xlabel('time [s]');
ylabel('vehicle position [m]');
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
title('vehicle velocities [m/s]');
xlabel('time [s]');
ylabel('vehicle velocities [m/s]');
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');


% figure(3);
% hplot = plot(plt.t, plt.a(1:7,:));
% set(hplot, 'LineWidth', 2);
% legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');

% figure(4);
% hplot = plot(plt.t, plt.a(10:11,:));
% set(hplot, 'LineWidth', 2);
% legend('Aa', 'Ama');
    

figure(5)
hplot = plot(plt.t, plt.v_distance);
set(hplot, 'LineWidth', 2);
title('vehicle distance to goal in Action 1 [m]');
xlabel('time [s]');
ylabel('distance [m]');
legend('dx', 'dy', 'dz');
    
figure(6)
hplot = plot(plt.t, plt.v_misal);
set(hplot, 'LineWidth', 2);
title('vehicle misalignment to goal in Action 1 [rad]');
xlabel('time [s]');
ylabel('misalignment [rad]');
legend('rho_x', 'rho_y', 'rho_z');
 
figure(7)
hplot = plot(plt.t, plt.Horizontal);
set(hplot, 'LineWidth', 2);
title('Horizontal misalignment in Action 1 and 2 [rad]');
xlabel('time [s]');
ylabel('misalignment [rad]');
legend('rho_x', 'rho_y', 'rho_z');

figure(8)
hplot = plot(plt.t, plt.altitude);
set(hplot, 'LineWidth', 2);
title('altitude in action 1 and 2 [m]');
xlabel('time [s]');
ylabel('altitude [m]');

figure(9)
hplot = plot(plt.t, plt.Headingtorock);
set(hplot, 'LineWidth', 2);
title('misalignment respect the nodule in Action 2 [rad]');
xlabel('time [s]');
ylabel('misalignment [rad]');
legend('thetax', 'thetay', 'thetaz');

figure(10)
hplot = plot(plt.t, plt.Distancetorock);
set(hplot, 'LineWidth', 2);
title('distance respect the nodule in Action 2 [m]');
xlabel('time [s]');
ylabel('distance [m]');

figure(11)
hplot = plot(plt.t, plt.Distancetorockee);
set(hplot, 'LineWidth', 2);
title('distance respect the nodule of the end effector in Action 3 [m]');
xlabel('time [s]');
ylabel('distance [m]');

figure(12)
hplot = plot(plt.t, plt.Headingtorockee);
set(hplot, 'LineWidth', 2);
title('misalignment respect the nodule of the end effector in Action 3 [rad]');
xlabel('time [s]');
ylabel('misalignment [rad]');
legend('thetax', 'thetay', 'thetaz');

figure(13)
hplot = plot(plt.t, plt.mu);
set(hplot, 'LineWidth', 2);
title('mu parameter for the arm dexterity in Action 3');
xlabel('time [s]');
ylabel('mu [-]');


end

