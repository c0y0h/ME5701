function [] = plotObstacles(obstacle, obstacleR)
%PLOTOBSTACLES Summary of this function goes here
%   Detailed explanation goes here

theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
    x = obstacleR * cos(theta) + obstacle(id,1);
    y = obstacleR * sin(theta) + obstacle(id,2);
    plot(x,y,'-m');
    hold on;
end

end

