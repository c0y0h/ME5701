function [EvalDB] = NormalizeEval(EvalDB)
%NORMALIZEEVAL Summary of this function goes here
%   Detailed explanation goes here
% do normalization
if sum(EvalDB(:,3)) ~= 0
    EvalDB(:,3) = EvalDB(:,3)/sum(EvalDB(:,3));
end

if sum(EvalDB(:,4)) ~= 0
    EvalDB(:,4) = EvalDB(:,4)/sum(EvalDB(:,4));
end

if sum(EvalDB(:,5)) ~= 0
    EvalDB(:,5) = EvalDB(:,5)/sum(EvalDB(:,5));
end

end

