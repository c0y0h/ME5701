function [u,trajDB] = DWA(x,model,goal,evalParam,ob,R)
%DWA Summary of this function goes here
%   input:  current state, model param, goal, score function param, 
%           ob position, ob radius
%   output: control signal, possible traj

% calculate dynamic window according to current state
%   dynamic window = [vmin, vmax, wmin, wmax]
Vr = CalcDynamicWindow(x,model);

% get evalDB and trajDB
%   evalDB: every row [speed, angular speed, heading score, distance score, speed score]
%   trajDB: one trajectory for every 5 rows
[evalDB,trajDB] = Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    disp('no path to goal!');
    u=[0;0];
    return;
end

% normalization for further socre
evalDB = NormalizeEval(evalDB);

% n possible trajectory for scoring
feval = [];
for id=1:length(evalDB(:,1))
    feval = [feval; evalParam(1:3)*evalDB(id,3:5)'];    % evalParam is the weights
end
evalDB = [evalDB feval];    % add final socre for each traj in last column of evalDB

% choose control signal with the highest score
[maxScore, ind] = max(feval);
u = evalDB(ind,1:2)';

end

