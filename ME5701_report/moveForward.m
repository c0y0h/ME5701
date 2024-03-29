function [x] = moveForward(x,u)
%MOVEFORWARD Summary of this function goes here
%   Detailed explanation goes here

global dt;

F = [1   0   0   0   0;
     0   1   0   0   0;
     0   0   1   0   0;
     0   0   0   0   0;
     0   0   0   0   0];

B = [dt*cos(x(3))   0;
     dt*sin(x(3))   0;
     0             dt;
     1              0;
     0              1];

x = F*x + B*u;

end

