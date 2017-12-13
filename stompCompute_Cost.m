function Cost = stompCompute_Cost(nthetap, obst, hole, Env)
%Compute the cost of all points on one path

[~, W] = size(nthetap);
Cost = zeros(1, W);
vel = zeros(120, 1);
[X, ~] = updateQ(nthetap(:, 1));
[robotF, robotR] = stompRobot_Formation(X);
Cost(1) = Cost(1) + stompCost_obstacle(robotF, robotR, Env, vel);
for i = 2 : W
    robotFO = robotF;
    [X, ~] = updateQ(nthetap(:, i));
    [robotF, robotR] = stompRobot_Formation(X);
    vel = sqrt(sum((robotF - robotFO).^2, 2));
    %Cost(i) = Cost(i) + stompCollision_Cost(robotF, robotR, obst, hole);
    Cost(i) = Cost(i) + stompCost_obstacle(robotF, robotR, Env, vel);
end
    
end

