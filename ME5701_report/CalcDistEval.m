function [dist] = CalcDistEval(x,ob,R)
%CALCDISTEVAL Summary of this function goes here
%   Detailed explanation goes here
dist = 100;
for io=1:length(ob(:,1))
    tmp = norm(ob(io,:) - x(1:2)') - R;
    if dist > tmp
        dist = tmp;
    end
end

% upper limit
if dist >= 2*R
    dist = 2*R;
end

end

