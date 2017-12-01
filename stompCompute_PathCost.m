function Cost = stompCompute_PathCost(theta, env, R)
%Compute the overall cost of a path

Costi = stompCompute_Cost(theta, env);
Cost = sum(Costi) + 1/2 * theta' * R * theta;

end