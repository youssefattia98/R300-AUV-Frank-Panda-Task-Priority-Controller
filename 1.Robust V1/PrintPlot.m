function [ ] = PrintPlot( plt )
x=plt.action_transition_time;
% %Vehicle position and velocity (p and v)
% figure(1);
% subplot(2,1,1);
% hplot = plot(plt.t, plt.p);
% title('Vehicle Position [m]');
% xlabel('time [s]');
% ylabel('Vehicle Position [m]');
% set(hplot, 'LineWidth', 1);
% legend('x','y','z','roll','pitch','yaw');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% subplot(2,1,2);
% hplot = plot(plt.t, plt.p_dot);
% title('Vehicle Velocities [m/s]');
% xlabel('time [s]');
% ylabel('Vehicle Velocities [m/s]');
% set(hplot, 'LineWidth', 1);
% legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
%   
% 
% %Arm configuration and velocity (q and q_dot)
% figure(2);
% subplot(2,1,1);
% hplot = plot(plt.t, plt.q);
% title('Arm Configuration [rad]');
% xlabel('time [s]');
% ylabel('Arm Configuration [rad]');
% set(hplot, 'LineWidth', 1);
% legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% subplot(2,1,2);
% hplot = plot(plt.t, plt.q_dot);
% title('Arm Configuration Speed [rad/s]');
% xlabel('time [s]');
% ylabel('Arm configuration Speed [rad/s]');
% set(hplot, 'LineWidth', 1);
% legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% 
% %Vehicle Altitiude and Attitude
% figure(3)
% subplot(2,1,1);
% hplot = plot(plt.t, plt.altitude);
% set(hplot, 'LineWidth', 2);
% title('Vechicle Altitiude [m]');
% xlabel('time [s]');
% ylabel('Vechicle Altitiude [m]');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% subplot(2,1,2);
% hplot = plot(plt.t, plt.Horizontal);
% set(hplot, 'LineWidth', 2);
% title('Vechicle Attitude [rad]');
% xlabel('time [s]');
% ylabel('Vechicle Attitude [rad]');
% legend('rho_x', 'rho_y', 'rho_z');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% 
% 
% 
% %Navigate to goal: Vehicle distance and misalignment to goal (d and p)
% figure(4)
% subplot(2,1,1);
% hplot = plot(plt.t, plt.v_distance);
% set(hplot, 'LineWidth', 2);
% title('Navigate to goal: Vehicle Distance To Goal [m]');
% xlabel('time [s]');
% ylabel('Navigate to goal: Vehicle Distance To Goal [m]');
% legend('dx', 'dy', 'dz');
% hold on;
% for i = 1:size(plt.action_transition_time)
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% 
% subplot(2,1,2);
% hplot = plot(plt.t, plt.v_misal);
% set(hplot, 'LineWidth', 2);
% title('Navigate to goal: Vehicle Misalignment With Goal [rad]');
% xlabel('time [s]');
% ylabel('Navigate to goal: Vehicle Misalignment With Goal [rad]');
% legend('rho_x', 'rho_y', 'rho_z');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% 
% 
% 
%Landing: Vehicle distance and misalignment to nodule (d and p)
figure(5)
% subplot(2,1,1);
% hplot = plot(plt.t, plt.Distancetorock);
% set(hplot, 'LineWidth', 2);
% title('Landing: Vehicle Distance To Nodule [m] , gain = 0.01');
% xlabel('time [s]');
% ylabel('Landing: Vehicle Distance To Nodule [m]');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% subplot(2,1,2);
hplot = plot(plt.t, plt.Headingtorock);
set(hplot, 'LineWidth', 2);
title('Landing: Vehicle misalignment With Nodule[rad] , gain = 0.01');
%title('Landing: Vehicle misalignment With Nodule[rad]');
xlabel('time [s]');
ylabel('Landing: Vehicle misalignment With Nodule [rad]');
legend('thetax', 'thetay', 'thetaz');
hold on;
for i = 1:2
    vline(plt.action_transition_time(i),'r');
end
hold off;
% 
% 
% %Inspection: Arm distance and misalignment to nodule (d and p)
% figure(6)
% subplot(2,1,1);
% hplot = plot(plt.t, plt.Distancetorockee);
% set(hplot, 'LineWidth', 2);
% title('Inspection: Arm Distance To Nodule [m]');
% xlabel('time [s]');
% ylabel('Inspection: Arm Distance To Nodule [m]');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
% subplot(2,1,2);
% hplot = plot(plt.t, plt.Headingtorockee);
% set(hplot, 'LineWidth', 2);
% title('Inspection: Arm Misalignment With Nodule [rad]');
% xlabel('time [s]');
% ylabel('Inspection: Arm Misalignment With Nodule [rad]');
% legend('thetax', 'thetay', 'thetaz');
% hold on;
% for i = 1:2
%     vline(plt.action_transition_time(i),'r');
% end
% hold off;
end

