function [pandaArm] = ComputeJacobians(pandaArm)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, -0.0305686, 1.53975, 0.753872] ) 
%
% remember: the control vector is:
% [q_dot] 
% [qdot_1, qdot_2, ..., qdot_7]
%
% therefore all task jacobians should be of dimensions
% m x 14
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]

% Left Arm base to ee Jacobian
pandaArm.ArmL.bJe = geometricJacobian(pandaArm.ArmL.franka,[pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
% Left Arm base to ee Jacobian
pandaArm.ArmR.bJe = geometricJacobian(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT
% Common Jacobians
pandaArm.Jjl = eye(14);


% Left Arm world to tool Jacobian:
pandaArm.ArmL.Ste = [eye(3) zeros(3);  -skew(pandaArm.ArmL.wTe(1:3,1:3)*pandaArm.ArmL.eTt(1:3,4)) eye(3)];
pandaArm.ArmL.Jw_wt = pandaArm.ArmL.Ste * [pandaArm.ArmL.wTb(1:3,1:3) zeros(3,3); zeros(3,3) pandaArm.ArmL.wTb(1:3,1:3)] * pandaArm.ArmL.bJe(:, 1:7);

% Right Arm world to tool Jacobian:
pandaArm.ArmR.Ste = [eye(3) zeros(3);  -skew(pandaArm.ArmR.wTe(1:3,1:3)*pandaArm.ArmR.eTt(1:3,4)) eye(3)];
pandaArm.ArmR.Jw_wt = pandaArm.ArmR.Ste * [pandaArm.ArmR.wTb(1:3,1:3) zeros(3,3); zeros(3,3) pandaArm.ArmR.wTb(1:3,1:3)] * pandaArm.ArmR.bJe(:, 1:7);

% Common jacobian:
pandaArm.Jt = [pandaArm.ArmL.Jw_wt zeros(6, 7); zeros(6, 7) pandaArm.ArmR.Jw_wt];


% joint limit jacobian:
pandaArm.ArmL.Jjl = eye(7);
pandaArm.ArmR.Jjl = eye(7);

% Common jacobian:
pandaArm.Jjl = [pandaArm.ArmL.Jjl zeros(7); zeros(7) pandaArm.ArmR.Jjl];


% Kinematic constraint jacobian:
% Left Arm world to object Jacobian:
pandaArm.ArmL.Sot = [eye(3) zeros(3);  -skew(pandaArm.ArmL.wTt(1:3,1:3)*pandaArm.ArmL.tTo(1:3,4)) eye(3)];
pandaArm.ArmL.Jw_o = pandaArm.ArmL.Sot * pandaArm.ArmL.Jw_wt(:, 1:7);

% Right Arm world to object Jacobian:
pandaArm.ArmR.Sot = [eye(3) zeros(3);  -skew(pandaArm.ArmR.wTe(1:3,1:3)*pandaArm.ArmR.tTo(1:3,4)) eye(3)];
pandaArm.ArmR.Jw_o = pandaArm.ArmR.Sot * pandaArm.ArmR.Jw_wt(:, 1:7);

% Common jacobian:
pandaArm.Jo = [pandaArm.ArmL.Jw_o, -pandaArm.ArmR.Jw_o];


% Reaching goal1 jacobian
pandaArm.Jg1 = [[zeros(3, 7); pandaArm.ArmL.Jw_wt(4:6, :)] zeros(6, 7); zeros(6, 7) [zeros(3, 7); pandaArm.ArmR.Jw_wt(4:6, :)]];

% finish jacobian
pandaArm.Jf = pandaArm.Jt;


end