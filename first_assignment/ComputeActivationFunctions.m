function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here


if (mission.prev_action == "go_to")
    previous_tasks = mission.actions.go_to.tasks;
elseif (mission.prev_action == "landing")
    previous_tasks = mission.actions.landing.tasks;
elseif (mission.prev_action == "reachingrock")
    previous_tasks = mission.actions.reachingrock.tasks;
end

if (mission.current_action == "go_to")
    current_tasks = mission.actions.go_to.tasks;
elseif (mission.current_action == "landing")
    current_tasks = mission.actions.landing.tasks;
elseif (mission.current_action == "reachingrock")
    current_tasks = mission.actions.reachingrock.tasks;
end

% arm tool position control
% always active
%uvms.A.t = eye(6).* ActionTransition("T", previous_tasks, current_tasks, mission.phase_time);
uvms.A.v_d = eye(3) .* ActionTransition("V_D", previous_tasks, current_tasks, mission.phase_time);
uvms.A.v_o = eye(3) .* ActionTransition("V_O", previous_tasks, current_tasks, mission.phase_time);
uvms.A.ha = IncreasingBellShapedFunction(0.25, 0.5, 0, 1, uvms.theta_ha) .* ActionTransition("HA", previous_tasks, current_tasks, mission.phase_time);
uvms.A.ma = DecreasingBellShapedFunction(0.5, 1, 0, 1, uvms.altitude) .* ActionTransition("MA", previous_tasks, current_tasks, mission.phase_time);
uvms.A.a = ActionTransition("A", previous_tasks, current_tasks, mission.phase_time);
uvms.A.hr = eye(3) .* ActionTransition("HR", previous_tasks, current_tasks, mission.phase_time);

rock_center = [12.2025   37.3748  -39.8860  1]'; % in world frame coordinates
v_r1 = uvms.vTw * rock_center; % project the rock position in the vehicle reference frame (whit homogeneus coordinates)
v_r = v_r1(1:3, 1); % put it back in normal coordinates = distance from the origin of the vehicle reference frame and the rock
w_kw = [0 0 1]'; % define the k-unit vector of the world reference frame
v_kw = uvms.vTw(1:3,1:3) * w_kw; % evaluate the projection of the k-unit vector of the world reference frame
v_rp = v_r - v_kw * dot(v_kw, v_r); % evaluate the projection of the distance between the vehicle and the rock on the horizontal plane
uvms.A.dr = eye(2) .* IncreasingBellShapedFunction(1.1, 1.6, 0, 1, norm(v_rp)) .* ActionTransition("DR", previous_tasks, current_tasks, mission.phase_time);
uvms.A.fbt = eye(3) .* ActionTransition("FBT", previous_tasks, current_tasks, mission.phase_time);
uvms.A.fbr = eye(3) .* ActionTransition("FBR", previous_tasks, current_tasks, mission.phase_time);
uvms.A.rrt = eye(3) .* ActionTransition("RRT", previous_tasks, current_tasks, mission.phase_time);
uvms.A.rrr = eye(3) .* ActionTransition("RRR", previous_tasks, current_tasks, mission.phase_time);

ujl = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];
for i = 1:size(ujl, 1)
    ArrayActivationFunction1(i, i) = IncreasingBellShapedFunction(uvms.jlmax(i) - 0.01, uvms.jlmax(i), 0, 1, uvms.q(i));
end
uvms.A.ujl = ArrayActivationFunction1 .* ActionTransition("UJL", previous_tasks, current_tasks, mission.phase_time);

ljl = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
for i = 1:size(ljl, 1)
    ArrayActivationFunction2(i, i) = DecreasingBellShapedFunction(uvms.jlmin(i), uvms.jlmin(i) + 0.01, 0, 1, uvms.q(i));
end
uvms.A.ljl = ArrayActivationFunction2 .* ActionTransition("LJL", previous_tasks, current_tasks, mission.phase_time);

uvms.A.dex = DecreasingBellShapedFunction(0, 0.1, 0, 1, uvms.mu) .* ActionTransition("DEX", previous_tasks, current_tasks, mission.phase_time);