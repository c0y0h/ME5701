function [evalDB,trajDB] = Evaluation(x,Vr,goal,ob,R,model,evalParam)
%EVALUATION Summary of this function goes here
%   input:  current state, window, goal, obs position, obs radius, evalParam
%   output: evalDB: 5 rows, [speed, angular speed, heading score, distance score, speed score]
%           trajDB: one trajectory every 5 rows, 31 points for every trajectory

evalDB = [];
trajDB = [];

for vt=Vr(1):model(5):Vr(2)     % all possible speed
    for ot=Vr(3):model(6):Vr(4) % all possible angular speed
        % generate trajectory
        %   xt is the predicted state after moving forward;
        %   traj is the trajctory between current time and next time
        [xt, traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);
        
        % calculate all socre functions
        heading = CalcHeadingEval(xt,goal);     % heading score: less delta heading, higher score
        dist = CalcDistEval(xt,ob,R);           % distance score: closer to goal, higher score
        vel = abs(vt);                          % speed score: faster is better
        stopDist = CalcBreakingDist(vel,model);    % breaking length
        if dist > stopDist  % distance should larger than breaking length
            evalDB = [evalDB; [vt, ot, heading, dist, vel]];
            trajDB = [trajDB; traj];
        end
    end
end

end

