function Cost = stompCompute_Cost(ntheta, obst, hole)
%Compute the cost of all points on one path
Cost = 0;

[X, ~] = updateQ(ntheta);
[robotF, robotR] = stompRobot_Formation(X);
Cost = Cost + stompCollision_Cost(robotF, robotR, obst, hole);

end

