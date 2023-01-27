function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

ujl = [];
ljl = [];

% to be computed at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;
uvms.vTgv = eye(4,4);
uvms.wTgv = eye(4,4);

uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
uvms.Jv_d = [];
uvms.Jv_o = [];
uvms.Jha = [];
uvms.Ja = [];
uvms.Jma = [];
uvms.Jv_hr = [];
uvms.Jv_fbt = [];
uvms.Jv_fbr = [];
uvms.Jv_rrt = [];
uvms.Jv_rrr = [];
uvms.Jv_ujl = [];
uvms.Jv_ljl = [];
uvms.Jv_dex = [];


uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
uvms.xdot.v_d = [];
uvms.xdot.v_o = [];
uvms.xdot.ha = [];
uvms.xdot.a = [];
uvms.xdot.ma = [];
uvms.xdot.v_hr = [];
uvms.xdot.fbt = [];
uvms.xdot.fbr = [];
uvms.xdot.rrt = [];
uvms.xdot.rrr = [];
uvms.xdot.ujl = [];
uvms.xdot.ljl = [];
uvms.xdot.dex = [];

    
uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.ha = zeros(1,1);
uvms.A.t = zeros(6,6);
uvms.A.v_d = zeros(3);
uvms.A.v_o = zeros(3);
uvms.A.ha = 0;
uvms.A.a = 0;
uvms.A.ma = 0;
uvms.A.hr = 0;
uvms.A.fbt = 0;
uvms.A.fbr = 0;
uvms.A.rrt = 0;
uvms.A.rrr = 0;
uvms.A.ujl = 0;
uvms.A.ljl = 0;
uvms.A.dex = 0;



uvms.v_misal = zeros(3,1);
uvms.v_distance = zeros(3,1);
uvms.v_n_ha = zeros(3,1);
uvms.theta_ha = 0;
uvms.altitude = 0;
end

