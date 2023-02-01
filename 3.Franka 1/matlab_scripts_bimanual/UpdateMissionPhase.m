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
                active = 0;
                for i = 1:7
                    active = active + abs(pandaArm.ArmL.A.jl(i, i)) + abs(pandaArm.ArmR.A.jl(i, i));
                end
                if (active > 0.1)
                    mission.phase = 3;
                    mission.previous_action = mission.current_action;
                    mission.current_action = "finish";
                    mission.phase_time = 0;
                end
            case 3 % Finish motion
                     
        end
end

