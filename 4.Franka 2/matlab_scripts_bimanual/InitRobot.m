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
pandaArm.ArmL.wTb = wTb1;
pandaArm.ArmR.wTb = wTb2;
pandaArm.ArmL.wTe = pandaArm.ArmL.wTb*pandaArm.ArmL.bTe;
pandaArm.ArmR.wTe = pandaArm.ArmR.wTb*pandaArm.ArmR.bTe;

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
% Internal evaluations matices:
pandaArm.ArmL.Ste = [];
pandaArm.ArmR.Ste = [];
pandaArm.ArmL.eTt = [rotation(0, 0, -43.1937 * ((2 * pi)/360)) [0; 0; 0.1]; 0 0 0 1];
pandaArm.ArmR.eTt = [rotation(0, 0, -43.1937 * ((2 * pi)/360)) [0; 0; 0.1]; 0 0 0 1];

% Transformation matices:
pandaArm.ArmL.wTt = [];
pandaArm.ArmR.wTt = [];
pandaArm.ArmL.wTg = [];
pandaArm.ArmR.wTg = [];
pandaArm.ArmL.tTo = [];
pandaArm.ArmR.tTo = [];
pandaArm.ArmL.wTo = [];
pandaArm.ArmR.wTo = [];
pandaArm.ArmL.wTg1 = [];
pandaArm.ArmR.wTg1 = [];
pandaArm.ArmL.w_rho_Leeg = [];
pandaArm.ArmL.w_dist_Leeg = [];
pandaArm.ArmR.w_rho_Reeg = [];
pandaArm.ArmR.w_dist_Reeg = [];
pandaArm.ArmL.w_rho_Leeg1 = [];
pandaArm.ArmL.w_dist_Leeg1 = [];
pandaArm.ArmR.w_rho_Reeg1 = [];
pandaArm.ArmR.w_dist_Reeg1 = [];
pandaArm.ArmL.mu = 0;
pandaArm.ArmR.mu = 0;
pandaArm.x_dot.mu = [];
pandaArm.ArmL.x_dot.mu = [];
pandaArm.ArmR.x_dot.mu = [];
pandaArm.x_dot.fcv = [];
pandaArm.ArmL.x_dot.fcv = [];
pandaArm.ArmR.x_dot.fcv = [];

% Jacobians matrices:
pandaArm.ArmL.Jw_wt = [];
pandaArm.ArmR.Jw_wt = [];
pandaArm.Jt = [];
pandaArm.ArmL.Jjl = [];
pandaArm.ArmR.Jjl = [];
pandaArm.Jjl = [];
pandaArm.ArmL.Jw_o = [];
pandaArm.ArmR.Jw_o = [];
pandaArm.Jo = [];
pandaArm.Jg1 = [];
pandaArm.ArmL.Jf = [];
pandaArm.ArmL.Jf = [];

% Init Task Reference vectors
pandaArm.xdot.jl = [];
pandaArm.xdot.ArmL.tool = [];
pandaArm.xdot.ArmR.tool = [];

pandaArm.xdot.ArmL.rg = [];
pandaArm.xdot.ArmR.rg = [];
pandaArm.xdot.rg = [];
pandaArm.xdot.ArmL.jl = [];
pandaArm.xdot.ArmR.jl = [];
pandaArm.xdot.jl = [];
pandaArm.xdot.kc = [];
pandaArm.xdot.ArmL.rg1 = [];
pandaArm.xdot.ArmR.rg1 = [];
pandaArm.xdot.rg1 = [];
pandaArm.ArmL.xdot.f = [];
pandaArm.ArmR.xdot.f = [];

% Init Activation Functions
pandaArm.A.tool = zeros(6,6);
pandaArm.A.jl = zeros(14,14);

pandaArm.ArmL.A.rg = [];
pandaArm.ArmR.A.rg = [];
pandaArm.A.rg = [];
pandaArm.A.A1 = 0;
pandaArm.ArmL.A.jl = [];
pandaArm.ArmR.A.jl = [];
pandaArm.A.jl = [];
pandaArm.A.kc = [];
pandaArm.ArmL.A.rg1 = [];
pandaArm.ArmR.A.rg1 = [];
pandaArm.A.rg1 = [];
pandaArm.ArmL.A.f = [];
pandaArm.ArmR.A.f = [];

% Activation function for activate or deactivate tasks
pandaArm.Aa.tool = zeros(6,6);
pandaArm.Aa.jl = zeros(14,14);



end

