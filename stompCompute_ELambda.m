function E = stompCompute_ELambda(pathCost)
%Compute the exp(-1/lambda * S) form for Prob computation

h = 10;

maxS = max(pathCost, [], 1);
minS = min(pathCost, [], 1);

E = exp(-h * (pathCost - minS) ./ (maxS - minS));

end