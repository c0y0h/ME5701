function [stopDist] = CalcBreakingDist(vel,model)
%CALCBREAKINGDIST Summary of this function goes here
%   Detailed explanation goes here
global dt;
stopDist = 0;
while vel>0
    stopDist = stopDist + vel*dt;
    vel = vel - model(3)*dt;
end

end

