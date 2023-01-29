function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                if (norm(pandaArm.ArmL.w_rho_Leeg) <= 0.1 && norm(pandaArm.ArmL.w_dist_Leeg) <= 0.1 && norm(pandaArm.ArmR.w_rho_Reeg) <= 0.1 && norm(pandaArm.ArmR.w_dist_Reeg) <= 0.1)
                    mission.phase = 2;
                    mission.previous_action = mission.current_action;
                    mission.current_action = "grasping";
                    mission.phase_time = 0;
                end
            case 2 % Cooperative Manipulation Start
                if (norm(pandaArm.ArmL.q_dot) < 0.1 && norm(pandaArm.ArmR.q_dot))
                    mission.phase = 3;
                    mission.previous_action = mission.current_action;
                    mission.current_action = "finish";
                    mission.phase_time = 0;
                end
            case 3 % Finish motion
                     
        end
end

