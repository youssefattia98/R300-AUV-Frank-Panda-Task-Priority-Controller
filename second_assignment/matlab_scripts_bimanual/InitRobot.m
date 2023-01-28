function [pandaArm] = InitRobot(model,wTb1,wTb2)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArm containing the two robot
% model
pandaArm.ArmL = model;
pandaArm.ArmR = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArm.ArmL.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArm.ArmR.q = pandaArm.ArmL.q;
pandaArm.ArmL.q_dot = [0 0 0 0 0 0 0]';
pandaArm.ArmR.q_dot = [0 0 0 0 0 0 0]';
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka,[pandaArm.ArmL.q',0,0],'panda_link7');
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');

pandaArm.ArmR.wTb = wTb2; 
pandaArm.ArmL.wTb = wTb1;
pandaArm.ArmL.wTe = pandaArm.ArmL.wTb*pandaArm.ArmL.bTe;
pandaArm.ArmR.wTe = pandaArm.ArmR.wTb*pandaArm.ArmR.bTe;
pandaArm.ArmR.Ste = [];
pandaArm.ArmL.Ste = [];

% Define trasnformation matrix from ee to tool.
I = eye(4); % done damages, create an identity matrix 4x4
I(3,4) = 0.1; % done damages, set the translation of 10cm along z
pandaArm.ArmL.eTt = I; % done damages, set the result as transformation matrix
pandaArm.ArmR.eTt = I; % done damages, set the result as transfromation matrix

% joint limits corresponding to the actual Panda by Franka arm configuration
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
pandaArm.jlmin  = [jlmin; jlmin];
pandaArm.jlmax  = [jlmax; jlmax];

% Init relevance Jacobians
pandaArm.ArmL.bJe = eye(6,7);
pandaArm.ArmR.bJe = eye(6,7);
pandaArm.Jjl = [];

%% ... TO HERE
% Init Task Reference vectors
pandaArm.xdot.jl = [];
pandaArm.xdot.ArmL.tool = [];
pandaArm.xdot.ArmR.tool = [];

% Init Activation Functions
pandaArm.A.tool = zeros(6,6);
pandaArm.A.jl = zeros(14,14);

% Activation function for activate or deactivate tasks
pandaArm.Aa.tool = zeros(6,6);
pandaArm.Aa.jl = zeros(14,14);



end

