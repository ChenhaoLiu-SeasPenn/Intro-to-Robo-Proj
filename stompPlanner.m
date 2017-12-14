clear all;close all;

%%
%Parameters
T = 5;
nSamples = 100;
kPaths = 20;
convThr = 0;

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
% Env_edt = sEDT_3d(Env);

%%
%Initialization
TStart = [0 1 0 130; 0 0 1 180; 1 0 0 280; 0 0 0 1];
TGoal = [1 0 0 263.5; 0 1 0 -50; 0 0 1 122.25; 0 0 0 1];
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
Q_time = [];
RAR_time = [];
Qtheta = stompCompute_PathCost(theta, obsts, hole, R, Env_edt);
QthetaOld = 0;
tic
ite=0;
while abs(Qtheta - QthetaOld) > convThr
    ite=ite+1;
%     Qtheta
    QthetaOld = Qtheta;
    
    %Random Sampling
    [ntheta, epsilon] = stompCompute_NoisyTraj(kPaths,qStart,qGoal,Rinv, theta);
    %Compute Cost and Probability
    pathCost = zeros(kPaths, nSamples);
    pathE = zeros(kPaths, nSamples);
    pathProb = zeros(kPaths, nSamples);
    for i = 1 : kPaths
        pathCost(i, :) = stompCompute_Cost(ntheta{i}, obsts, hole, Env_edt);
    end
    pathE = stompCompute_ELambda(pathCost);
    pathProb = pathE ./ sum(pathE, 1);
    pathProb(isnan(pathProb) == 1) = 0;
    
    
    %Compute delta
    dtheta = sum(pathProb .* epsilon, 1);
    
    if sum(sum(pathCost)) == 0
        dtheta = zeros(nSamples);
    end
    
    %Smooth delta
    dtheta = M * dtheta(2 : nSamples - 1)';
    
    %Update theta
    theta(:, 2 : nSamples - 1) = theta(:, 2 : nSamples - 1) + [dtheta';dtheta';dtheta';dtheta';dtheta'];
%     theta
    
    %Compute new trajectory cost
    Qtheta = stompCompute_PathCost(theta, obsts, hole, R, Env_edt);
    
    if mod(ite, 5000) == 1
        fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)...
            Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3) Cube(1,3) Cube(1,3) Cube(1,3)], 'b');
        fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)...
            Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'b')
        for i= 1:100
            [X,~]=updateQ([theta(:,i)' 0]);
            plot3(X(1, 1), X(1, 2), X(1, 3), 'bo', 'markersize', 6);
            plot3(X(2, 1), X(2, 2), X(2, 3), 'ro', 'markersize', 6);
            plot3(X(3, 1), X(3, 2), X(3, 3), 'go', 'markersize', 6);
            plot3(X(4, 1), X(4, 2), X(4, 3), 'yo', 'markersize', 6);
            plot3(X(5, 1), X(5, 2), X(5, 3), 'ko', 'markersize', 6);
            plot3(X(6, 1), X(6, 2), X(6, 3), 'mo', 'markersize', 6);
            lynxServoSim([theta(:,i)' 0]);
%             pause(0.01);
        end
%         if mod(ite, 50) == 1
%             close
%             lynxStart(); hold on;
%         end
    end
    Qtheta
    Q_time = [Q_time Qtheta];
    RAR = 1/2 * sum(sum(theta(:, 2:99) * R * theta(:, 2:99)'));
    RAR_time = [RAR_time RAR];
    if ite > 1000 || sum(sum(dtheta)) == 0
        break
    end
end
disp('We finished!!!!!!!!!!!!!!!');
toc
%%
%Visualization
% plotObstacle([140 140;180 180;280 280],35,1);
% plotObstacle([220 220;100 100;200 200],35,1);
disp(['iteration:',num2str(ite)]);


% fill3([100 100 1000 1000],[-1000 1000 1000 -1000],[], 'r')
 fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)... 
     Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3) Cube(1,3) Cube(1,3) Cube(1,3)], 'b');
  fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)... 
     Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'b')
for i= 1: length(theta)
    [X,~]=updateQ([theta(:,i)' 0]);
    plot3(X(1, 1), X(1, 2), X(1, 3), 'bo', 'markersize', 6);
    plot3(X(2, 1), X(2, 2), X(2, 3), 'ro', 'markersize', 6);
    plot3(X(3, 1), X(3, 2), X(3, 3), 'go', 'markersize', 6);
    plot3(X(4, 1), X(4, 2), X(4, 3), 'yo', 'markersize', 6);
    plot3(X(5, 1), X(5, 2), X(5, 3), 'ko', 'markersize', 6);
    plot3(X(6, 1), X(6, 2), X(6, 3), 'mo', 'markersize', 6);
    lynxServoSim([theta(:,i)' 0]);
    pause(0.01);
end

figure;
plot(1:ite, Q_time, 'b', 'linewidth', 2);hold on;
figure;
plot(1:ite, RAR_time, 'r', 'linewidth', 2);
%%
%Actuate on Lynx