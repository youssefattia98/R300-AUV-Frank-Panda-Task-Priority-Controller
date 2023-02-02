function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                if (norm(pandaArm.ArmL.w_rho_Leeg) <= 0.1 && norm(pandaArm.ArmL.w_dist_Leeg) <= 0.1 && norm(pandaArm.ArmR.w_rho_Reeg) <= 0.1 && norm(pandaArm.ArmR.w_dist_Reeg) <= 0.1)
                    pandaArm.ta(1,1) = pandaArm.t;
                    mission.phase = 2;
                    mission.previous_action = mission.current_action;
                    mission.current_action = "grasping1";
                    mission.phase_time = 0;
                end
            case 2 % Cooperative Manipulation Start
                if (norm(pandaArm.ArmL.w_dist_Leeg1) <= 0.1 &&  norm(pandaArm.ArmR.w_dist_Reeg1) <= 0.1)
                    pandaArm.ta(1,2) = pandaArm.t;
                    mission.phase = 3;
                    mission.previous_action = mission.current_action;
                    mission.current_action = "grasping2";
                    mission.phase_time = 0;
                end
            case 3 % Finish motion
                 active = 0;
                for i = 1:7
                    active = active + abs(pandaArm.ArmL.A.jl(i, i)) + abs(pandaArm.ArmR.A.jl(i, i));
                end
                if (active > 0.1)
                    pandaArm.ta(1,3) = pandaArm.t;
                    mission.phase = 4;
                    mission.previous_action = mission.current_action;
                    mission.current_action = "finish";
                    mission.phase_time = 0;
                end
        end
end

