function Cost = stompCompute_PathCost(theta, obsts, hole, R, Env)
%Compute the overall cost of a path

theta = theta(:, 2:99);
Costi = stompCompute_Cost(theta, obsts, hole, Env);
Cost = sum(Costi) + 1/2 * sum(sum(theta * R * theta'));

end