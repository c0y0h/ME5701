function [x, traj] = GenerateTrajectory(x,vt,ot,evaldt,model)
%GENERATETRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
%   input:  current state, current speed, current acceleration, 
%           forward time, model param
%   output: xt:   the predicted state after moving forward;
%           traj: the trajctory between current time and next time (evaldt / dt + 1, here 31 points)

global dt;
time = 0;

u = [vt; ot];
traj = x;
while time <= evaldt
    time = time + dt;
    x = moveForward(x,u);
    traj = [traj, x];
end

end

