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
% Right Arm base to ee Jacobian
pandaArm.ArmR.bJe = geometricJacobian(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

% Left Arm world to ee jacobian
pandaArm.ArmL.wJe = pandaArm.ArmL.bJe;

%% Ste is the rigid body transformation from end-effector frame of the right arm to
% tool-frame frame projected on <w>
pandaArm.ArmR.Ste = [eye(3) zeros(3);  -skew(pandaArm.ArmR.wTe(1:3,1:3)*pandaArm.ArmR.eTt(1:3,4)) eye(3)];
% pandaArm.ArmR.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
pandaArm.ArmR.Jt_a  = pandaArm.ArmR.Ste * [pandaArm.ArmR.wTb(1:3,1:3) zeros(3,3); zeros(3,3) pandaArm.ArmR.wTb(1:3,1:3)] * pandaArm.ArmR.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
pandaArm.ArmR.Jt_v = [zeros(3) eye(3); eye(3) -skew(pandaArm.ArmR.eTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
pandaArm.ArmR.Jt = [pandaArm.ArmR.Jt_a pandaArm.ArmR.Jt_v]; % jacobian of the manipulator respect the vehicle reference frame

%% Ste is the rigid body transformation from end-effector frame of the right arm to
% tool-frame frame projected on <w>
pandaArm.ArmL.Ste = [eye(3) zeros(3);  -skew(pandaArm.ArmL.wTe(1:3,1:3)*pandaArm.ArmL.eTt(1:3,4)) eye(3)];
% pandaArm.ArmL.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
pandaArm.ArmL.Jt_a  = pandaArm.ArmL.Ste * [pandaArm.ArmL.wTb(1:3,1:3) zeros(3,3); zeros(3,3) pandaArm.ArmL.wTb(1:3,1:3)] * pandaArm.ArmL.bJe;
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
pandaArm.ArmL.Jt_v = [zeros(3) eye(3); eye(3) -skew(pandaArm.ArmL.eTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
pandaArm.ArmL.Jt = [pandaArm.ArmL.Jt_a pandaArm.ArmL.Jt_v]; % jacobian of the manipulator respect the vehicle reference frame

% Common Jacobians
pandaArm.Jjl = eye(14);


end