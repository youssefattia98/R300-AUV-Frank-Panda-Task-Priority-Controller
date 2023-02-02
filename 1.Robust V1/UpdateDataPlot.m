function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.toolPos(:, loop) = uvms.wTt(1:3,4);

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;

%plt.xdot_jl(:, loop) = uvms.xdot.jl;
%plt.xdot_mu(:, loop) = uvms.xdot.mu;
plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.t;

plt.a(1:7, loop) = diag(uvms.A.jl);
plt.a(8, loop) = uvms.A.mu;
plt.a(9, loop) = uvms.A.ha(1,1);
plt.a(10, loop) = uvms.A.a(1,1);
plt.a(11, loop) = uvms.A.ma(1,1);

plt.toolx(:,loop) = uvms.wTt(1,4);
plt.tooly(:,loop) = uvms.wTt(2,4);

plt.v_misal(:,loop) = uvms.v_misal;
plt.v_distance(:,loop) = uvms.v_distance;
plt.altitude(:,loop) = uvms.altitude;
plt.Horizontal(:,loop) = uvms.theta_ha;
plt.Headingtorock(:, loop) = uvms.v_rho_r;
plt.Distancetorock(:, loop) = norm(uvms.v_rp);
plt.Distancetorockee(:, loop) = uvms.rrt;
plt.Headingtorockee(:, loop) = uvms.rrr;
plt.mu(:, loop) = uvms.mu;
plt.action_transition_time =uvms.ta;


end