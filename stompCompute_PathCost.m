function Cost = stompCompute_PathCost(theta, obsts, hole, R)
%Compute the overall cost of a path

theta = theta(:, 2:99);
Costi = stompCompute_Cost(theta,  obsts, hole);
Cost = sum(Costi) + 1/2 * sum(sum(theta * R * theta'));

end