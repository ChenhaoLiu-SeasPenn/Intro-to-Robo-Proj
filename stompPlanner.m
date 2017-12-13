clear all;close all;

%%
%Parameters
T = 5;
nSamples = 100;
kPaths = 100;
convThr = 10;

%%
%Setup environment
lynxStart();hold on;
%Environment size
Env = zeros(100,100,100);
% %Obstacle cube
% obsts = [100 1000 -1000 1000 200 200];
obsts=[];
% %Passage hole [center r]
% hole = [0 0 200 60];
hole=[];
%Calculate EDT_Env
voxel_size = [10, 10, 10];
Env = constructEnv(voxel_size);
Env_edt = prod(voxel_size) ^ (1/3) * sEDT_3d(Env);

%%
%Initialization
TStart = [1 0 0 100; 0 1 0 100; 0 0 1 300; 0 0 0 1];
TGoal = [0 1 0 100; 0 0 1 -100; 1 0 0 100; 0 0 0 1];
qStart = IK_lynx(TStart);
qStart = qStart(1:5)
qGoal = IK_lynx(TGoal);
qGoal = qGoal(1:5)
theta = [linspace(qStart(1), qGoal(1), nSamples);linspace(qStart(2), qGoal(2), nSamples);linspace(qStart(3), qGoal(3), nSamples);...
    linspace(qStart(4), qGoal(4), nSamples);linspace(qStart(5), qGoal(5), nSamples)];
%Initialize theta on a line
ntheta = cell(kPaths, 1);

%%
%Precompute
A_k = eye(nSamples - 1, nSamples - 1);
A = -2 * eye(nSamples, nSamples);
A(1:nSamples - 1, 2:nSamples) = A(1:nSamples - 1, 2:nSamples) + A_k;
A(2:nSamples, 1:nSamples - 1) = A(2:nSamples, 1:nSamples - 1) + A_k;
A = A(:, 2:99);
R = A' * A;
Rinv = inv(R);
M = 1 / nSamples * Rinv ./ max(Rinv, [], 1);
Rinv = Rinv / sum(sum(Rinv));

%%
%Planner

Qtheta = stompCompute_PathCost(theta, obsts, hole, R, Env_edt);
QthetaOld = 0;

while abs(Qtheta - QthetaOld) > convThr
    Qtheta
    QthetaOld = Qtheta;
    
    %Random Sampling
    [ntheta, epsilon] = stompCompute_NoisyTraj(kPaths,qStart,qGoal,Rinv);
    %Compute Cost and Probability
    pathCost = zeros(kPaths, nSamples);
    pathE = zeros(kPaths, nSamples);
    pathProb = zeros(kPaths, nSamples);
    for i = 1 : kPaths
        pathCost(i, :) = stompCompute_Cost(ntheta{i}', obsts, hole, Env_edt);
    end
    pathE = stompCompute_ELambda(pathCost);
    pathProb = pathE ./ sum(pathE, 1);
    pathProb(isnan(pathProb) == 1) = 0.05;
    
    %Compute delta
    dtheta = sum(pathProb .* epsilon, 1);
    
    %Smooth delta
    dtheta = M * dtheta(2 : nSamples - 1)';
    
    %Update theta
    theta(:, 2 : nSamples - 1) = theta(:, 2 : nSamples - 1) + [dtheta';dtheta';dtheta';dtheta';dtheta'];
%     theta
    
    %Compute new trajectory cost
    Qtheta = stompCompute_PathCost(theta, obsts, hole, R, Env_edt);
    
end

%%
%Visualization
fill3([100 100 1000 1000],[-1000 1000 1000 -1000],[obsts(5) obsts(5) obsts(5) obsts(5)], 'r')
fill3([-60 -60 60 60], [-60 60 60 -60], [200 200 200 200], 'b')
for i= 1: length(theta)
    [X,~]=updateQ([theta(:,i)' 0]);
    plot3(X(1, 1), X(1, 2), X(1, 3), 'bo', 'markersize', 6);
    plot3(X(2, 1), X(2, 2), X(2, 3), 'ro', 'markersize', 6);
    plot3(X(3, 1), X(3, 2), X(3, 3), 'go', 'markersize', 6);
    plot3(X(4, 1), X(4, 2), X(4, 3), 'yo', 'markersize', 6);
    plot3(X(5, 1), X(5, 2), X(5, 3), 'ko', 'markersize', 6);
    plot3(X(6, 1), X(6, 2), X(6, 3), 'mo', 'markersize', 6);
    lynxServoSim([theta(:,i)' 0]);
end
%%
%Actuate on Lynx