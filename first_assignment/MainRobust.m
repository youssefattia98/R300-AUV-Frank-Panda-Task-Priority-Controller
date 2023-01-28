function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 100;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% rock position 
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
uAltitude.setup();

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('Robust');
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
%uvms.p = [8.5 38.5 -38   0 -0.06 0.5]'; 
uvms.p = [8.5 38.5 -36 0  -0.06 0.5]';


% defines the goal position for the end-effector/tool position task
uvms.goalPosition = [12.2025   37.3748  -39.8860]';
uvms.wRg = rotation(0, pi, pi/2);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% defines the goal position for the vehicle position task
%uvms.vehicleGoalPosition = [10.5 37.5 -38]';
uvms.vehicleGoalPosition = [10.5 37.5 -38]';

uvms.wRgv = rotation(0, 0, 0);
uvms.wTgv = [uvms.wRgv uvms.vehicleGoalPosition; 0 0 0 1];

% defines the tool control point
uvms.eTt = eye(4);

mission.prev_action = "go_to";
mission.current_action = "go_to";
mission.phase = 1;
mission.phase_time = 0;
mission.actions.go_to.tasks = ["MA", "HA", "V_O", "V_D"];
mission.actions.landing.tasks = ["HA", "HR", "DR", "A"];
mission.actions.reachingrock.tasks = ["UJL", "LJL", "DEX", "FBT", "FBR", "RRT", "RRR"];

tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
    
    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(13,1);
    Qp = eye(13); 
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    %[Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.ujl,    uvms.Jv_ujl,    Qp, ydotbar, uvms.xdot.ujl,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.ljl,    uvms.Jv_ljl,    Qp, ydotbar, uvms.xdot.ljl,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.dex,    uvms.Jv_dex,    Qp, ydotbar, uvms.xdot.dex,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.fbt,    uvms.Jv_fbt,    Qp, ydotbar, uvms.xdot.fbt,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.fbr,    uvms.Jv_fbr,    Qp, ydotbar, uvms.xdot.fbr,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.rrr,    uvms.Jv_rrr,    Qp, ydotbar, uvms.xdot.rrr,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.rrt,    uvms.Jv_rrt,    Qp, ydotbar, uvms.xdot.rrt,  0.0001,   0.01, 10);
    % arm prefered shape
    [Qp, ydotbar] = iCAT_task(uvms.A.ma,    uvms.Jma,    Qp, ydotbar, uvms.xdot.ma,  0.0001,   0.01, 10);    
    [Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.v_o,    uvms.Jv_o,    Qp, ydotbar, uvms.xdot.v_o,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.v_d,    uvms.Jv_d,    Qp, ydotbar, uvms.xdot.v_d,  0.0001,   0.01, 10); 
    [Qp, ydotbar] = iCAT_task(uvms.A.hr,    uvms.Jv_hr,    Qp, ydotbar, uvms.xdot.v_hr,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.dr,    uvms.Jv_dr,    Qp, ydotbar, uvms.xdot.v_dr,  0.0001,   0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.a,    uvms.Ja,    Qp, ydotbar, uvms.xdot.a,  0.0001,   0.01, 10);   
    
    
    %[....]
    [Qp, ydotbar] = iCAT_task(eye(13),     eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integration
    uvms.q_dot = ydotbar(1:7);
    uvms.p_dot = ydotbar(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        %uvms.sensorDistance
        
        act = uvms.A.ma
        act_a = uvms.A.a
        mission.phase
       
    end
    
    mission.phase_time = mission.phase_time + deltat;

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    %SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end