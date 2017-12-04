function [cost] = stompCollision_Cost(robotF, robotR, obs, hole)
%Parameters
r0 = 10;
r1 = 20;
costH = 100000;

%obstacle is a surface at z = 200
holeR = hole(4) - r0;

distRobot = abs(robotF(:, 3) - obs(5)) + (1000 * (robotF(:, 1) <= 60));
distHoleCenter = sqrt(sum((robotF(:, 1:2) - hole(:, 1:2)) .^ 2, 2));

cost = (distRobot < (robotR + r1)) * costH - ...
    0.05 * ((distRobot > (robotR + r0)) & (distRobot < (robotR + r1))) .* (distRobot - (robotR + r0)) .^ 2;

cost = sum(cost .* (distHoleCenter > holeR));

end