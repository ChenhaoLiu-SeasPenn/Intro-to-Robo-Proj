clear all;close all;

%%
%Parameters
T = 5;
nSamples = 100;
kPaths = 20;
convThr = 1e-4;

%%
%Setup environment



%%
%Initialization
xStart = [];
xGoal = [];
theta = zeros(nSamples, 3);
%Initialize theta on a line
theta = [];
ntheta = cell(kPaths, 1);

%%
%Precompute
A_k = eye(nSamples - 1, nSamples - 1);
A = -2 * eye(nSamples, nSamples);
A(1:nSamples - 1, 2:nSamples) = A(1:nSamples - 1, 2:nSamples) + A_k;
A(2:nSamples, 1:nSamples - 1) = A(2:nSamples, 1:nSamples - 1) + A_k;
A(1, 1) = -1;
A(nSamples, nSamples) = -1;
R = A' * A;
Rinv = inv(R);
M = Rinv;

%%
%Planner

Qtheta = stompCompute_PathCost(theta, env, R);
QthetaOld = 0;

while Qtheta - QthetaOld < convThr
    QthetaOld = Qtheta;
    
    %Random Sampling
    [ntheta, epsilon] = stompCompute_Random(ntheta, kPaths, Rinv);

    %Compute Cost and Probability
    pathCost = zeros(kPaths, nSamples);
    pathE = zeros(kPaths, nSamples);
    pathProb = zeros(kPaths, nSamples);
    for i = 1 : kPaths
        pathCost(i, :) = stompCompute_Cost(ntheta{i}, env);
    end
    pathE = stompCompute_ELambda(pathCost);
    pathProb = pathE ./ sum(pathE, 1);

    %Compute delta
    dtheta = sum(pathProb .* epsilon, 1);
    
    %Smooth delta
    dtheta = M * dtheta;
    
    %Update theta
    theta(2 : nSamples - 1) = theta(2 : nSamples - 1) + dtheta(2 : nSamples - 1);
    
    %Compute new trajectory cost
    Qtheta = stompCompute_PathCost(theta, env, R);
end

%%
%Visualization

%%
%Actuate on Lynx