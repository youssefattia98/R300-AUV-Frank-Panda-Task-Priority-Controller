function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from end-effector frame to
% tool-frame frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];

% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v]; % jacobian of the manipulator respect the vehicle reference frame

% Jacobian for the vehicle distance task
uvms.Jv_d = [zeros(3,7) -eye(3) zeros(3)];

% Jacobian for the vehicle orientation task
uvms.Jv_o = [zeros(3,7) zeros(3) eye(3)];

% Jacobian for the horizontal attitude task
uvms.Jha = [zeros(1,7) zeros(1,3) uvms.v_n_ha'];

% Jacobian for the (minimum) altitude task
w_kw = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;
uvms.Ja = [zeros(1,7) v_kw' zeros(1,3)];
uvms.Jma = uvms.Ja;

% Jacobian for the heading to rock task
P = eye(3) - uvms.vTw(1:3, 1:3) * [0 0 0; 0 0 0; 0 0 1];
rock_center = [12.2025   37.3748  -39.8860  1]'; % in world frame coordinates
v_r1 = uvms.vTw * rock_center; % project the rock position in the vehicle reference frame (whit homogeneus coordinates)
v_r = v_r1(1:3, 1); % put it back in normal coordinates = distance from the origin of the vehicle reference frame and the rock
w_kw = [0 0 1]'; % define the k-unit vector of the world reference frame
v_iv = [1 0 0]'; % define the x-unit vector of the vehicle reference frame
v_kw = uvms.vTw(1:3,1:3) * w_kw; % evaluate the projection of the k-unit vector of the world reference frame
v_rp = v_r - v_kw * dot(v_kw, v_r); % evaluate the projection of the distance between the vehicle and the rock on the horizontal plane 
v_rho_r = ReducedVersorLemma(v_iv, v_rp); % evaluate the misalignment between the projection of the distance on the horizontal plane and the x-axis of the vehicle
n = v_rho_r ./ norm(v_rho_r);
Pd = P * (v_r);
Pd2 = norm(v_r1) * norm(v_r1);
Pscrew = [0 -Pd(1, 1) Pd(2, 1); Pd(3, 1) 0 -Pd(1, 1); -Pd(2, 1) Pd(1, 1) 0];
uvms.Jv_hr = [zeros(3, 7), (-1 / Pd2) * Pscrew * P' -eye(3)];
hr = uvms.Jv_hr
%uvms.Jv_hr = uvms.Jv_o;
% Jacobian for the distance to rock task
tj = uvms.vTw * [(-(eye(3) - [0 0 0; 0 0 0; 0 0 1])) [0; 0; 0]; [0 0 0 1]];
uvms.Jv_dr = [zeros(2, 7) tj(1:2, 1:3) zeros(2, 3)];

% Jacobian for the fix-base rotational task
uvms.Jv_fb = [zeros(6, 7) eye(6)];

% Jacobian for the reaching the rock with the end effector task (translational part):
uvms.Jv_rrt = uvms.Jt(4:6, :);

% Jacobian for the reaching the rock with the end effector task (rotational part):
uvms.Jv_rrr = uvms.Jt(1:3, :);

% Jacobian for the joint limit task:
uvms.Jv_jl = [eye(7) zeros(7, 6)];

% Jacobian for avoiding singularity/Dexterity/manipulability task
[Jmu, uvms.mu] = ComputeManipulability(uvms.bJe, uvms.djdq);
uvms.Jv_dex = [Jmu zeros(1,6)];
end