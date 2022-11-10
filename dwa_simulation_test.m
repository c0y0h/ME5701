%   Dynamic Window Approach with different filters
 
close all;
clear all;

disp('Simulation start!');

% some index definitions
POS_X = 1;     % pos_x
POS_Y = 2;     % pos_y
YAW = 3;       % yaw
V_SPD = 4;     % speed
W_SPD = 5;     % angular speed

MD_MAX_V = 1;           % maximum speed
MD_MAX_W = 2;           % maximum angular speed
MD_ACC = 3;             % acceleration 
MD_VW = 4;              % angular acceleration
MD_V_RESOLUTION = 5;    % speed resolution
MD_W_RESOLUTION = 6;    % angular speed resolution


% initial states x of robot: [pos_x, pos_y, yaw, v, w]
x = [0 0 pi/10 0 0]';

% robot kinematic parameters: [max_v, max_w, a, aw, vresolution, wresolution]
kinematic = [1.0, deg2rad(20.0), 0.2, deg2rad(50.0), 0.01, deg2rad(1)];

% goal position
goal = [10, 10];

% obstacle list
obstacle = [0 2;
            2 5;
            4 2;
            5 4;
            5 6;
            5 9;
            8 8;
            8 9;
            7 9];
% obstacle radius
obstacleR = 0.5;

% time
global dt;
dt = 0.1;   % time interval

% simulation area
area = [-1 11 -1 11];

% parameters for trajectory scores: 
% [head_score_weight, distance_score_weight, speed_score_weight, forward time]
evalParam = [0.05, 0.2, 0.1, 3.0];

% results of trajectory point states
result.x = [];
tic;

Xplus = zeros(5,5000);
Xplus(:,1) = x;
Pplus1 = 0.1;

% main loop
for i=2:5000
    % required control signal u = [v, w] and trajectory 
    [u, traj] = DWA(x, kinematic, goal, evalParam, obstacle, obstacleR);
    % cannot accurately get to the position
    real_x = moveForward(x,u) + [normrnd(0,0.01,2,1);0;0;0];

    % prediction equation
    gpsxy = [1 0 0 0 0;
             0 1 0 0 0]*real_x + normrnd(0,0.05,2,1);

    % EKF to estimate x,y
    Q = [0.1   0 0 0 0;
           0 0.1 0 0 0;
           0   0 0 0 0;
           0   0 0 0 0;
           0   0 0 0 0];
    R = [0.1   0;
           0 0.1];
    % predict
    A = [1, 0, -dt*Xplus(4,i-1)*sin(Xplus(3,i-1)), dt*cos(Xplus(3,i-1)), 0;
         0, 1,  dt*Xplus(4,i-1)*cos(Xplus(3,i-1)), dt*sin(Xplus(3,i-1)), 0;
         0, 0,  1,                                 0,                   dt;
         0, 0,  0,                                 1,                    0;
         0, 0,  0,                                 0,                    1];
    


    % store the trajectory
    result.x = [result.x; x'];

    % reach the goal
    if norm(x(POS_X:POS_Y) - goal') < 0.1
        disp('Reach the destination');
        break;
    end

    % Visualization
    hold off;
    ArrowLength = 0.5;
    quiver(x(POS_X), x(POS_Y), ArrowLength*cos(x(YAW)), ArrowLength*sin(x(YAW)), 'ok');  % robot
    hold on;
    plot(result.x(:,POS_X), result.x(:,POS_Y), '-b');   % plot history trajectory points
    hold on;
    plot(goal(1), goal(2), '*r');   % plot goal postiion
    hold on;
    plotObstacles(obstacle, obstacleR); % plot all obstacles
    % plot trajectories for evaluation
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5    % every 5 rows represent one traj
            ind = 1+(it-1)*5;           % index of the it th traj
            plot(traj(ind,:), traj(ind+1,:),'-g');
            hold on;
        end
    end
    axis(area); % the simulation area
    grid on;
    drawnow;    % refresh
end
toc % the running time















