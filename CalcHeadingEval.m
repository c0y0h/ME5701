function [heading] = CalcHeadingEval(x,goal)
%CALCHEADINGEVAL Summary of this function goes here
%   Detailed explanation goes here

theta = rad2deg(x(3));
goalTheta = rad2deg(atan2(goal(2) - x(2), goal(1) - x(1)));

if goalTheta > theta
    targetTheta = goalTheta - theta;
else
    targetTheta = theta - goalTheta;
end

heading = 180 - targetTheta;

end

