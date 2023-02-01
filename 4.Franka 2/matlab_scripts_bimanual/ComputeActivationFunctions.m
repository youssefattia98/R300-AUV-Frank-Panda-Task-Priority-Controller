function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

        switch mission.phase
            case 1  % REACH GOAL FRAME
                previous_tasks = mission.actions.reach_goal.tasks;
                current_tasks = mission.actions.reach_goal.tasks;

            case 2 % GRASP AND MOVE THE OBJECT
                previous_tasks = mission.actions.reach_goal.tasks;
                current_tasks = mission.actions.grasping.tasks;

            case 3 % STOP any motion 
                previous_tasks = mission.actions.grasping.tasks;
                current_tasks = mission.actions.finish.tasks;
        end

% Reaching Goal (orientation and position)
pandaArm.ArmL.A.rg = eye(6) .* ActionTransition("RGL", previous_tasks, current_tasks, mission.phase_time);
pandaArm.ArmR.A.rg = eye(6) .* ActionTransition("RGR", previous_tasks, current_tasks, mission.phase_time);

% Joint Limits Task
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
for i = 1 : 7
    pandaArm.ArmL.A.jl(i, i) = IncreasingBellShapedFunction(jlmax(i) - 0.1, jlmax(i), 0, 1, pandaArm.ArmL.q(i)) + DecreasingBellShapedFunction(jlmin(i), jlmin(i) + 0.1, 0, 1,  pandaArm.ArmL.q(i));
end
for i = 1 : 7
    pandaArm.ArmR.A.jl(i, i) = IncreasingBellShapedFunction(jlmax(i) - 0.1, jlmax(i), 0, 1, pandaArm.ArmR.q(i)) + DecreasingBellShapedFunction(jlmin(i), jlmin(i) + 0.1, 0, 1,  pandaArm.ArmR.q(i));
end
pandaArm.ArmL.A.jl = pandaArm.ArmL.A.jl .* ActionTransition("JLL", previous_tasks, current_tasks, mission.phase_time);
pandaArm.ArmR.A.jl = pandaArm.ArmR.A.jl .* ActionTransition("JLR", previous_tasks, current_tasks, mission.phase_time);

% Reaching Goal1 (orientation and position)
pandaArm.ArmL.A.rg1 = eye(6) .* ActionTransition("RG1L", previous_tasks, current_tasks, mission.phase_time);
pandaArm.ArmR.A.rg1 = eye(6) .* ActionTransition("RG1R", previous_tasks, current_tasks, mission.phase_time);


% finish task
pandaArm.ArmL.A.f = eye(6) .* ActionTransition("FL", previous_tasks, current_tasks, mission.phase_time);
pandaArm.ArmR.A.f = eye(6) .* ActionTransition("FR", previous_tasks, current_tasks, mission.phase_time);


end
