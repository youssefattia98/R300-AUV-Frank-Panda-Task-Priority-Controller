function [pandaArm] = UpdateTransforms(pandaArm)
% the function updates all the transformations

% Left arm transformations
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka,[pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT

% Right arm transformations
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

% Transformation matices:
pandaArm.ArmL.wTe = pandaArm.ArmL.wTb*pandaArm.ArmL.bTe;
pandaArm.ArmR.wTe = pandaArm.ArmR.wTb*pandaArm.ArmR.bTe;
pandaArm.ArmL.wTt = pandaArm.ArmL.wTe * pandaArm.ArmL.eTt;
pandaArm.ArmR.wTt = pandaArm.ArmR.wTe * pandaArm.ArmR.eTt;
pandaArm.ArmL.wTo = pandaArm.ArmL.wTt * pandaArm.ArmL.tTo;
pandaArm.ArmR.wTo = pandaArm.ArmR.wTt * pandaArm.ArmR.tTo;



