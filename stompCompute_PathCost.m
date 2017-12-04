function Cost = stompCompute_PathCost(theta, obsts, hole, R)
%Compute the overall cost of a path

Costi = stompCompute_Cost(theta,  obsts, hole);
Cost = sum(Costi) + 1/2 * theta' * R * theta;

end