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
wTb1 = eye(4); % wTb1 here i'm doing damages
wRb = rotation(0, 0, pi); % done damages, define the rotation matrix between w=b1 and b2
wtb = [1.05; 0; 0]; % done damages, definig the translation along x of 105cm
wtb_1 = cat(2,wRb,wtb); % done damages, first concatenation
wTb2 = cat(1,wtb_1,[0, 0, 0, 1]); % wTb2 here i'm doing damages
pandaArm.ArmR.wTb = wTb2;

plt = InitDataPlot(maxloops);
pandaArmBimanual = InitRobot(model,wTb1,wTb2);
% Init object and tools frames
obj_length = 0.1;
w_obj_pos = [0.50735 0 0.4]';
w_obj_ori = rotation(0,0,0);

%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points.
%pandaArmBimanual.ArmL.wTg = ...;
%pandaArmBimanual.ArmR.wTg = ...;
% Second goal move the object
%pandaArmBimanual.wTog ...;

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
%     [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.jl,     pandaArmBimanual.Jjl,    Qp, ydotbar, [pandaArmBimanual.ArmL.xdot.jl;pandaArmBimanual.ArmR.xdot.jl],  0.0001,   0.01, 10);% JOINT LIMITS TASK
%     [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.tool,     pandaArmBimanual.ArmL.Jtool,    Qp, ydotbar, pandaArmBimanual.xdot.ArmL.tool,  0.0001,   0.01, 10);% LEFT ARM TOOL CONTROL TASK 
%     [Qp, ydotbar] = iCAT_task(pandaArmBimanual.A.tool,     pandaArmBimanual.ArmR.Jtool,    Qp, ydotbar, pandaArmBimanual.xdot.ArmR.tool,  0.0001,   0.01, 10);% RIGHT ARM TOOL CONTROL TASK 
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
        phase = mission.phase
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    SlowdownToRealtime(deltat);
    
end
PrintPlot(plt, pandaArmBimanual);
end
