function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% reference for the vehicle position and orientation tasks
[v_rho, v_dist] = CartError(uvms.vTgv, eye(4));

uvms.xdot.v_d = 0.2 * (zeros(3,1) - v_dist);
uvms.xdot.v_d = Saturate(uvms.xdot.v_d, 0.4);
uvms.xdot.v_o = 0.2 * (v_rho);
uvms.v_misal = v_rho;
uvms.v_distance = v_dist;

% reference for the horizontal attitude task
v_kv = [0 0 1]';
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;
v_rho_ha = ReducedVersorLemma(v_kw, v_kv);
uvms.theta_ha = norm(v_rho_ha);
if (uvms.theta_ha ~= 0)
    uvms.v_n_ha = v_rho_ha ./ uvms.theta_ha;
else
    uvms.v_n_ha = zeros(3,1);
end
uvms.xdot.ha = 0.2 * (0.25 - uvms.theta_ha);

% reference for the minimum altitude task
uvms.xdot.ma = 0.3 * (1 - uvms.altitude);

% reference for the altitude task
uvms.xdot.a = 0.3 * (0.1 - uvms.altitude);

% reference for the heading to rock task:
rock_center = [12.2025   37.3748  -39.8860  1]'; % in world frame coordinates
v_r1 = uvms.vTw * rock_center; % project the rock position in the vehicle reference frame (whit homogeneus coordinates)
v_r = v_r1(1:3, 1); % put it back in normal coordinates = distance from the origin of the vehicle reference frame and the rock
w_kw = [0 0 1]'; % define the k-unit vector of the world reference frame
v_iv = [1 0 0]'; % define the x-unit vector of the vehicle reference frame
v_kw = uvms.vTw(1:3,1:3) * w_kw; % evaluate the projection of the k-unit vector of the world reference frame
v_rp = v_r - v_kw * dot(v_kw, v_r); % evaluate the projection of the distance between the vehicle and the rock on the horizontal plane 
v_rho_r = ReducedVersorLemma(v_iv, v_rp); % evaluate the misalignment between the projection of the distance on the horizontal plane and the x-axis of the vehicle
% 0.01, 0.2, 0.8
uvms.xdot.v_hr = 0.01 * (-v_rho_r); % compute the task reference
uvms.v_rho_r = v_rho_r;

% reference for the distance to rock task:
uvms.xdot.v_dr = 0.2 * -((v_rp(1:2) - uvms.vTb(1:2, 4)) ./ norm(v_rp(1:2) - uvms.vTb(1:2, 4))) * (norm(v_rp(1:2) - uvms.vTb(1:2, 4)) - 0.5); %(v_rp(1:2) ./ norm(v_rp)) * (norm(v_rp(1:2)) - 1.2)
uvms.v_rp = v_rp(1:2);

% reference for the fixwed base task:
uvms.xdot.fb = [0 0 0 0 0 0]';

% reference for the reaching the rock with the end effector task (translational part):
ee = uvms.vTe(1:3, 4);
uvms.xdot.rrt = 0.2 * (v_r - ee);
uvms.rrt = norm(v_r - ee);

% reference for the reaching the rock with the end effector task (rotational part):
w_kw = [0 0 1]'; % define the k-unit vector of the world reference frame
v_kw = uvms.vTw(1:3,1:3) * (-w_kw); % evaluate the projection of the k-unit vector of the world reference frame
v_zee = uvms.vTe(1:3, 3);
v_rho_eekw = ReducedVersorLemma(v_kw, v_zee);
uvms.xdot.rrr = 0.2 * (-v_rho_eekw);
uvms.rrr = v_rho_eekw;

% reference for the joint limits task:
uvms.xdot.jl = zeros(7, 1);
for i = 1:size(uvms.jlmin)
    mean(i) = uvms.jlmin(i) + uvms.jlmax(i);
    if uvms.q(i) <= mean(i)
        uvms.xdot.jl(i) = 0.2 * (uvms.jlmin(i) - uvms.q(i) +0.01);
    else
        uvms.xdot.jl(i) = 0.2 * (uvms.jlmax(i) - uvms.q(i) -0.01);
    end
end

% reference for avoiding singularity/Dexterity/manipulability task
uvms.xdot.dex = 0.2 * (0.1 - uvms.mu);