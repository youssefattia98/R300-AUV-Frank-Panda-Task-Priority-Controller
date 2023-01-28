function [pandaArm] = UpdateTransforms(pandaArm)
% the function updates all the transformations

% Left arm transformations
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka,[pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
pandaArm.ArmL.wTe = pandaArm.ArmL.bTe;
pandaArm.ArmL.wTt = pandaArm.ArmL.eTt * pandaArm.ArmL.wTe;

% Right arm transformations
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT
pandaArm.ArmR.wTe = pandaArm.ArmR.wTb * pandaArm.ArmR.bTe;
pandaArm.ArmR.wTt = pandaArm.ArmR.eTt * pandaArm.ArmR.wTe;


