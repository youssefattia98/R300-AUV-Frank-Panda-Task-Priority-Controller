function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            % add policy for changing phase
            % computing the errors for the go-to action defining tasks
            [v_rho, v_dist] = CartError(uvms.vTgv, eye(4));
            if (norm(v_rho) < 0.01 && norm(v_dist) < 0.1)
                mission.phase = 2;
                mission.previous_action = mission.current_action;
                mission.current_action = "landing";
                mission.phase_time = 0;
            end
        case 2
            % add policy for changing phase
            % computing the errors for the landing action defining tasks
            rock_center = [12.2025   37.3748  -39.8860  1]'; % in world frame coordinates
            v_r1 = uvms.vTw * rock_center; % project the rock position in the vehicle reference frame (whit homogeneus coordinates)
            v_r = v_r1(1:3, 1); % put it back in normal coordinates = distance from the origin of the vehicle reference frame and the rock
            w_kw = [0 0 1]'; % define the k-unit vector of the world reference frame
            v_iv = [1 0 0]'; % define the x-unit vector of the vehicle reference frame
            v_kw = uvms.vTw(1:3,1:3) * w_kw; % evaluate the projection of the k-unit vector of the world reference frame
            v_rp = v_r - v_kw * dot(v_kw, v_r); % evaluate the projection of the distance between the vehicle and the rock on the horizontal plane 
            v_rho_r = ReducedVersorLemma(v_iv, v_rp); % evaluate the misalignment between the projection of the distance on the horizontal plane and the x-axis of the vehicle
            if (norm(v_rho_r) < 0.01 && abs(uvms.altitude - 0.1) < 0.01)
                mission.phase = 3;
                mission.previous_action = mission.current_action;
                mission.current_action = "reachingrock";
                mission.phase_time = 0;
            end
    end
end

