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
uvms.A.fb = eye(3) .* ActionTransition("FB", previous_tasks, current_tasks, mission.phase_time);
uvms.A.rr = eye(3) .* ActionTransition("RR", previous_tasks, current_tasks, mission.phase_time);