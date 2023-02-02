function MainPandaArmBimanual
addpath('./simulation_scripts');
clc;
clear;
close all

%% Initialization - DON'T CHANGE ANYTHING
% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 30;
loop = 1;
maxloops = ceil(end_time/deltat);
mission.phase = 1;
mission.phase_time = 0;
model = load("panda.mat");%load the model - DO NOT CHANGE panda.mat

% UDP Connection with Franka Interface - DO NOT CHANGE
hudps = dsp.UDPSender('RemoteIPPort',1500);
hudps.RemoteIPAddress = '127.0.0.1';

% Init robot model
wTb1 = eye(4); % fixed transformation word -> base1 (Left arm)
wTb2 = [rotation(0, 0, pi) [1.05; 0; 0]; [0 0 0 1]]; % fixed transformation word -> base2 (Right arm)
plt = InitDataPlot(maxloops);
pandaArmBimanual = InitRobot(model,wTb1,wTb2);
% Init object and tools frames
obj_length = 0.1;
w_obj_pos = [0.50735 0 0.4]';
w_obj_ori = rotation(0,0,0);

% Define trasnformation matrix from ee to tool.
pandaArmBimanual.ArmL.eTt = [rotation(0, 0, -43.1937 * ((2 * pi)/360)) [0; 0; 0.1]; 0 0 0 1];
pandaArmBimanual.ArmR.eTt = [rotation(0, 0, -43.1937 * ((2 * pi)/360)) [0; 0; 0.1]; 0 0 0 1];

%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points.
pandaArmBimanual.ArmL.wTg = [rotation(0, pi, pi) [0.45; 0; 0.59]; 0 0 0 1];
pandaArmBimanual.ArmR.wTg = [rotation(0, pi, 0) [0.55; 0; 0.59]; 0 0 0 1];
% Second goal move the object
pandaArmBimanual.ArmL.tTo = [rotation(pi, 0, 0) [0.05; 0; 0]; 0 0 0 1];
pandaArmBimanual.ArmR.tTo = [rotation(0, pi, 0) [0.05; 0; 0]; 0 0 0 1];
pandaArmBimanual.ArmL.wTg1 = [1 0 0 0.5; 0 1 0 -0.5; 0 0 1 0.5; 0 0 0 1];
pandaArmBimanual.ArmR.wTg1 = [1 0 0 0.5; 0 1 0 -0.5; 0 0 1 0.5; 0 0 0 1];
% third goal move to another point
pandaArmBimanual.ArmL.wTg2 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
pandaArmBimanual.ArmR.wTg2 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

%% Scheduler
mission.prev_action = "reach_goal";
mission.current_action = "reach_goal";
mission.phase = 1;
mission.phase_time = 0;
mission.actions.reach_goal.tasks = ["JL", "RG"];
mission.actions.grasping1.tasks = ["KC", "JL", "RG1"];
mission.actions.grasping2.tasks = ["KC", "JL", "RG2"];
mission.actions.finish.tasks = ["KC", "JL", "F"];

%% CONTROL LOOP

for t = 0:deltat:end_time
    % update all the involved variables
    pandaArmBimanual = UpdateTransforms(pandaArmBimanual);
    pandaArmBimanual = ComputeJacobians(pandaArmBimanual);
    pandaArmBimanual = ComputeActivationFunctions(pandaArmBimanual,mission);
    pandaArmBimanual = ComputeTaskReferences(pandaArmBimanual,mission);

    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(14,1);
    Qp = eye(14);
    % ADD minimum distance from table
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.kc,     pandaArmBimanual.Jo,    Qp, ydotbar, pandaArmBimanual.xdot.kc,  0.0001,   0.01, 10); % kinematic constraint
    [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.jl,     pandaArmBimanual.Jjl,    Qp, ydotbar, pandaArmBimanual.xdot.jl,  0.0001,   0.01, 10); % joint limit
    [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.f,     pandaArmBimanual.Jf,    Qp, ydotbar, pandaArmBimanual.xdot.f,  0.0001,   0.01, 10); % finish task
    [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.rg2,     pandaArmBimanual.Jg2,    Qp, ydotbar, pandaArmBimanual.xdot.rg2,  0.0001,   0.01, 10); % reaching goal 2 (orientation and position)
    [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.rg1,     pandaArmBimanual.Jg1,    Qp, ydotbar, pandaArmBimanual.xdot.rg1,  0.0001,   0.01, 10); % reaching goal 1 (orientation and position)
    [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.rg,     pandaArmBimanual.Jt,    Qp, ydotbar, pandaArmBimanual.xdot.rg,  0.0001,   0.01, 10); % reaching goal (orientation and position)
    [Qp, ydotbar] = iCAT_task(eye(14),     eye(14),    Qp, ydotbar, zeros(14,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integration
    pandaArmBimanual.ArmL.q_dot = ydotbar(1:7);
    pandaArmBimanual.ArmR.q_dot = ydotbar(8:14);
    % Integration
	pandaArmBimanual.ArmL.q = pandaArmBimanual.ArmL.q(1:7) + pandaArmBimanual.ArmL.q_dot*deltat;    
    pandaArmBimanual.ArmR.q = pandaArmBimanual.ArmR.q(1:7) + pandaArmBimanual.ArmR.q_dot*deltat;
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [pandaArmBimanual,mission] = UpdateMissionPhase(pandaArmBimanual, mission);
    %Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    posMsg = [pandaArmBimanual.ArmL.q;pandaArmBimanual.ArmR.q];
    step(hudps,posMsg)
    % Update data plot
    plt = UpdateDataPlot(plt,pandaArmBimanual,t,loop, mission);
    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        pandaArmBimanual.t = t;
        phase = mission.phase
        
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    SlowdownToRealtime(deltat);
    
end
PrintPlot(plt, pandaArmBimanual);
end
