function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

        switch mission.phase
            case 1  % REACH GOAL FRAME
                previous_tasks = mission.actions.reach_goal.tasks;
                current_tasks = mission.actions.reach_goal.tasks;

            case 2 % GRASP AND MOVE THE OBJECT 1
                previous_tasks = mission.actions.reach_goal.tasks;
                current_tasks = mission.actions.grasping1.tasks;

            case 3 % GRASP AND MOVE THE OBJECT 1
                previous_tasks = mission.actions.grasping1.tasks;
                current_tasks = mission.actions.grasping2.tasks;

            case 4 % STOP any motion 
                previous_tasks = mission.actions.grasping2.tasks;
                current_tasks = mission.actions.finish.tasks;
        end

% Control tasks (equality)
%pandaArm.A.tool = eye(6)*pandaArm.Aa.tool;

% Reaching Goal (orientation and position)
pandaArm.A.rg = eye(12) .* ActionTransition("RG", previous_tasks, current_tasks, mission.phase_time);

% Joint Limits Task
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
for i = 1 : 7
    pandaArm.ArmL.A.jl(i, i) = IncreasingBellShapedFunction(jlmax(i) - 0.1, jlmax(i), 0, 1, pandaArm.ArmL.q(i)) + DecreasingBellShapedFunction(jlmin(i), jlmin(i) + 0.1, 0, 1,  pandaArm.ArmL.q(i));
end
for i = 1 : 7
    pandaArm.ArmR.A.jl(i, i) = IncreasingBellShapedFunction(jlmax(i) - 0.1, jlmax(i), 0, 1, pandaArm.ArmR.q(i)) + DecreasingBellShapedFunction(jlmin(i), jlmin(i) + 0.1, 0, 1,  pandaArm.ArmR.q(i));
end
pandaArm.A.jl = [pandaArm.ArmL.A.jl zeros(7); zeros(7) pandaArm.ArmR.A.jl] .* ActionTransition("JL", previous_tasks, current_tasks, mission.phase_time);

% Kinematic Constraint Task
pandaArm.A.kc = eye(6) .* ActionTransition("KC", previous_tasks, current_tasks, mission.phase_time);

% Reaching Goal1 (orientation and position)
pandaArm.A.rg1 = eye(12) .* ActionTransition("RG1", previous_tasks, current_tasks, mission.phase_time); 

% Reaching Goal2 (orientation and position)
pandaArm.A.rg2 = eye(12) .* ActionTransition("RG2", previous_tasks, current_tasks, mission.phase_time); 

% finish task
pandaArm.A.f = eye(12) .* ActionTransition("F", previous_tasks, current_tasks, mission.phase_time); 


end
