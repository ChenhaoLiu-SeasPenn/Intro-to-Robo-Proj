function Cost = stompCompute_Cost(nthetap, obst, hole)
%Compute the cost of all points on one path

[~, W] = size(nthetap);
Cost = zeros(1, W);
for i = 1 : W
    [X, ~] = updateQ(nthetap(:, i));
    [robotF, robotR] = stompRobot_Formation(X);
    Cost(i) = Cost(i) + stompCollision_Cost(robotF, robotR, obst, hole);
end
    
end

