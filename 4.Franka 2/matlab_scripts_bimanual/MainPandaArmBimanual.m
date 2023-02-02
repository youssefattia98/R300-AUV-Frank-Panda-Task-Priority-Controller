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
pandaArmBimanual.ArmL.wTg2 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
pandaArmBimanual.ArmR.wTg2 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

%% Scheduler
mission.prev_action = "reach_goal";
mission.current_action = "reach_goal";
mission.phase = 1;
mission.phase_time = 0;
mission.actions.reach_goal.tasks = ["JLL", "JLR", "RGL", "RGR"];
mission.actions.grasping1.tasks = ["JLL", "JLR", "RG1L", "RG1R"];
mission.actions.grasping2.tasks = ["JLL", "JLR", "RG2L", "RG2R"];
mission.actions.finish.tasks = ["F"];

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
    
    ydotbari = zeros(7,1);
    Qpi = eye(7);
    % ADD minimum distance from table
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    
    [QpLs, ydotbarLs] = iCAT_task(pandaArmBimanual.ArmL.A.jl,     pandaArmBimanual.ArmL.Jjl,    Qpi, ydotbari, pandaArmBimanual.xdot.ArmL.jl,  0.0001,   0.01, 10); % joint limit Left arm
    [QpLs, ydotbarLs] = iCAT_task(pandaArmBimanual.ArmL.A.f,     pandaArmBimanual.ArmL.Jf,    QpLs, ydotbarLs, pandaArmBimanual.ArmL.xdot.f,  0.0001,   0.01, 10); % finish task
    [QpLs, ydotbarLs] = iCAT_task(pandaArmBimanual.ArmL.A.rg,     pandaArmBimanual.ArmL.Jw_wt,    QpLs, ydotbarLs, pandaArmBimanual.xdot.ArmL.rg,  0.0001,   0.01, 10); % reaching goal left arm (orientation and position)
    [QpLs, ydotbarLs] = iCAT_task(pandaArmBimanual.ArmL.A.rg1,     pandaArmBimanual.ArmL.Jw_o,    QpLs, ydotbarLs, pandaArmBimanual.xdot.ArmL.rg1,  0.0001,   0.01, 10); % reaching goal 1 Left arm (orientation and position)
    [QpLs, ydotbarLs] = iCAT_task(pandaArmBimanual.ArmL.A.rg2,     pandaArmBimanual.ArmL.Jw_o,    QpLs, ydotbarLs, pandaArmBimanual.xdot.ArmL.rg2,  0.0001,   0.01, 10); % reaching goal 2 Left arm (orientation and position)

    [QpRs, ydotbarRs] = iCAT_task(pandaArmBimanual.ArmR.A.jl,     pandaArmBimanual.ArmR.Jjl,    Qpi, ydotbari, pandaArmBimanual.xdot.ArmR.jl,  0.0001,   0.01, 10); % joint limit Right arm
    [QpRs, ydotbarRs] = iCAT_task(pandaArmBimanual.ArmR.A.f,     pandaArmBimanual.ArmR.Jf,    QpRs, ydotbarRs, pandaArmBimanual.ArmR.xdot.f,  0.0001,   0.01, 10); % finish task
    [QpRs, ydotbarRs] = iCAT_task(pandaArmBimanual.ArmR.A.rg,     pandaArmBimanual.ArmR.Jw_wt,    QpRs, ydotbarRs, pandaArmBimanual.xdot.ArmR.rg,  0.0001,   0.01, 10); % reaching goal (orientation and position)
    [QpRs, ydotbarRs] = iCAT_task(pandaArmBimanual.ArmR.A.rg1,     pandaArmBimanual.ArmR.Jw_o,    QpRs, ydotbarRs, pandaArmBimanual.xdot.ArmR.rg1,  0.0001,   0.01, 10); % reaching goal 1 Right arm (orientation and position)
    [QpRs, ydotbarRs] = iCAT_task(pandaArmBimanual.ArmR.A.rg2,     pandaArmBimanual.ArmR.Jw_o,    QpRs, ydotbarRs, pandaArmBimanual.xdot.ArmR.rg2,  0.0001,   0.01, 10); % reaching goal 2 Right arm (orientation and position)

    pandaArmBimanual.ArmL.x_dot.mu = pandaArmBimanual.ArmL.Jw_o * ydotbarLs;
    pandaArmBimanual.ArmR.x_dot.mu = pandaArmBimanual.ArmR.Jw_o * ydotbarRs;
    mu_o = 0.0001;
    pandaArmBimanual.ArmL.mu = mu_o + norm(pandaArmBimanual.ArmL.A.rg1 - pandaArmBimanual.ArmL.x_dot.mu);
    pandaArmBimanual.ArmR.mu = mu_o + norm(pandaArmBimanual.ArmR.A.rg1 - pandaArmBimanual.ArmR.x_dot.mu);
    pandaArmBimanual.x_dot.mu = (pandaArmBimanual.ArmL.mu * pandaArmBimanual.ArmL.x_dot.mu + pandaArmBimanual.ArmR.mu * pandaArmBimanual.ArmR.x_dot.mu) / (pandaArmBimanual.ArmL.mu + pandaArmBimanual.ArmR.mu);
    pandaArmBimanual.x_dot.fcv = pandaArmBimanual.H * (eye(12) - pandaArmBimanual.iC * pandaArmBimanual.C) * [pandaArmBimanual.x_dot.mu; pandaArmBimanual.x_dot.mu]; %feasible cooperative velocity vector
    pandaArmBimanual.ArmL.x_dot.fcv = pandaArmBimanual.x_dot.fcv(1:6, :);
    pandaArmBimanual.ArmR.x_dot.fcv = pandaArmBimanual.x_dot.fcv(7:12, :);
    
    [QpL, ydotbarL] = iCAT_task(pandaArmBimanual.ArmL.A.rg1,     pandaArmBimanual.ArmL.Jw_o,    Qpi, ydotbari, pandaArmBimanual.ArmL.x_dot.fcv,  0.0001,   0.01, 10); % reaching goal 1 Left arm (orientation and position) cooperative
    [QpL, ydotbarL] = iCAT_task(pandaArmBimanual.ArmL.A.rg2,     pandaArmBimanual.ArmL.Jw_o,    QpL, ydotbarL, pandaArmBimanual.ArmL.x_dot.fcv,  0.0001,   0.01, 10); % reaching goal 2 Left arm (orientation and position) cooperative
    [QpL, ydotbarL] = iCAT_task(pandaArmBimanual.ArmL.A.f,     pandaArmBimanual.ArmL.Jf,    QpL, ydotbarL, pandaArmBimanual.ArmL.xdot.f,  0.0001,   0.01, 10); % finish task
    [QpL, ydotbarL] = iCAT_task(pandaArmBimanual.ArmL.A.jl,     pandaArmBimanual.ArmL.Jjl,    QpL, ydotbarL, pandaArmBimanual.xdot.ArmL.jl,  0.0001,   0.01, 10); % joint limit Left arm
    [QpL, ydotbarL] = iCAT_task(pandaArmBimanual.ArmL.A.rg,     pandaArmBimanual.ArmL.Jw_wt,    QpL, ydotbarL, pandaArmBimanual.xdot.ArmL.rg,  0.0001,   0.01, 10); % reaching goal (orientation and position)
    [QpL, ydotbarL] = iCAT_task(eye(7),     eye(7),    QpL, ydotbarL, zeros(7,1),  0.0001,   0.01, 10);    % this task should be the last one

    [QpR, ydotbarR] = iCAT_task(pandaArmBimanual.ArmR.A.rg1,     pandaArmBimanual.ArmR.Jw_o,    Qpi, ydotbari, pandaArmBimanual.ArmR.x_dot.fcv,  0.0001,   0.01, 10); % reaching goal 1 Right arm (orientation and position)
    [QpR, ydotbarR] = iCAT_task(pandaArmBimanual.ArmR.A.rg2,     pandaArmBimanual.ArmR.Jw_o,    QpR, ydotbarR, pandaArmBimanual.ArmR.x_dot.fcv,  0.0001,   0.01, 10); % reaching goal 2 Right arm (orientation and position)
    [QpR, ydotbarR] = iCAT_task(pandaArmBimanual.ArmR.A.f,     pandaArmBimanual.ArmR.Jf,    QpR, ydotbarR, pandaArmBimanual.ArmR.xdot.f,  0.0001,   0.01, 10); % finish task
    [QpR, ydotbarR] = iCAT_task(pandaArmBimanual.ArmR.A.jl,     pandaArmBimanual.ArmR.Jjl,    QpR, ydotbarR, pandaArmBimanual.xdot.ArmR.jl,  0.0001,   0.01, 10); % joint limit Right arm
    [QpR, ydotbarR] = iCAT_task(pandaArmBimanual.ArmR.A.rg,     pandaArmBimanual.ArmR.Jw_wt,    QpR, ydotbarR, pandaArmBimanual.xdot.ArmR.rg,  0.0001,   0.01, 10); % reaching goal (orientation and position)
    [QpR, ydotbarR] = iCAT_task(eye(7),     eye(7),    QpR, ydotbarR, zeros(7,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    pandaArmBimanual.ArmL.x_dot.coop = pandaArmBimanual.ArmL.Jw_o * ydotbarL;
    pandaArmBimanual.ArmR.x_dot.coop = pandaArmBimanual.ArmR.Jw_o * ydotbarR;

    % get the two variables for integration
    pandaArmBimanual.ArmL.q_dot = ydotbarL;
    pandaArmBimanual.ArmR.q_dot = ydotbarR;
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
