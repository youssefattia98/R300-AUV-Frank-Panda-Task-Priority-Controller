function [A] = ActionTransition(taskname, previous, current, time)

if (ismember(taskname, previous) && ismember(taskname, current))
    A = 1;
elseif (ismember(taskname, previous) == 0 && ismember(taskname, current))
    A = IncreasingBellShapedFunction(0, 1, 0, 1, time);
elseif (ismember(taskname, previous) && ismember(taskname, current) == 0)
    A = DecreasingBellShapedFunction(0, 1, 0, 1, time);
else
    A = 0;

end

