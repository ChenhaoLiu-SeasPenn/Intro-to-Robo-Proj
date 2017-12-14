clear all;close all;

%%
%Parameters
T = 5;
nSamples = 100;
kPaths = 20;
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
[Env,Cube] = constructEnv(voxel_size);
Env_edt = prod(voxel_size) ^ (1/3) * sEDT_3d(Env);

%%
%Initialization
TStart = [1 0 0 240; 0 1 0 0; 0 0 1 180; 0 0 0 1];
TGoal = [1 0 0 163.5; 0 1 0 150; 0 0 1 240; 0 0 0 1];
qStart = IK_lynx(TStart);
qStart = qStart(1:5);
qGoal = IK_lynx(TGoal);
qGoal = qGoal(1:5);
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
tic
ite=0;
Qtheta_all=[];
while abs(Qtheta - QthetaOld) > convThr
    ite=ite+1;
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
    Qtheta
    Qtheta_all=[Qtheta_all Qtheta];
    
end
%Qtheta
disp('We finished!!!!!!!!!!!!!!!');
toc
%%
%Visualization
plotObstacle([140 140;180 180;280 280],35,1);
plotObstacle([220 220;100 100;100 100],35,1);
disp(['iteration:',num2str(ite)]);

%plot obstacle Cube
% fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)... 
%      Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3) Cube(1,3) Cube(1,3) Cube(1,3)], 'y');
% fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)... 
%      Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'y');
% fill3([Cube(1,1) Cube(1,1) Cube(1,1) Cube(1,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)... 
%      Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3) Cube(1,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'y');
% fill3([Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)... 
%      Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3) Cube(1,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'y');
% fill3([Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1) Cube(1,1)], [Cube(1,2) Cube(1,2)... 
%      Cube(1,2) Cube(1,2) ], [Cube(1,3) Cube(1,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'y');
% fill3([Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1) Cube(1,1)], [Cube(1,2)+Cube(2,2) Cube(1,2)+Cube(2,2)... 
%      Cube(1,2)+Cube(2,2) Cube(1,2)+Cube(2,2) ], [Cube(1,3) Cube(1,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'y');


for i= 1: length(theta)
    [X,~]=updateQ([theta(:,i)' 0]);
    plot3(X(1, 1), X(1, 2), X(1, 3), 'b.', 'markersize', 15);
    plot3(X(2, 1), X(2, 2), X(2, 3), 'r.', 'markersize', 15);
    plot3(X(3, 1), X(3, 2), X(3, 3), 'g.', 'markersize', 15);
    plot3(X(4, 1), X(4, 2), X(4, 3), 'y.', 'markersize', 15);
    plot3(X(5, 1), X(5, 2), X(5, 3), 'k.', 'markersize', 15);
    plot3(X(6, 1), X(6, 2), X(6, 3), 'm.', 'markersize', 15);
    lynxServoSim([theta(:,i)' 0]);
    pause(0.01);
end
figure;
plot(Qtheta_all);
ylabel('Overall cost')
xlabel('Iteration')


%%
%Actuate on Lynx