function [Vr] = CalcDynamicWindow(x,model)
%CALCDYNAMICWINDOW Summary of this function goes here
%   Detailed explanation goes here

global dt;

% vmin, vmax, wmin, wmax
Vs = [0, model(1), -model(2), model(2)];

% prediected vmin, vmax, wmin, wmax
Vd = [x(4)-model(3)*dt, x(4)+model(3)*dt, x(5)-model(4)*dt, x(5)+model(4)*dt];

Vtmp = [Vs;Vd];

Vr = [max(Vtmp(:,1)), min(Vtmp(:,2)), max(Vtmp(:,3)), min(Vtmp(:,4))];

end

