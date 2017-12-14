function [Traj_N,ek]=stompCompute_NoisyTraj(n_Samples,Start,End,R)
%THis function computes the noisy trajectory 

Num_k=n_Samples;
Traj=End-Start;
Step=[];
mu=zeros(1,length(R));
sigma=R;
Traj_N=cell(1,Num_k);

for i = 1 : length(R)
    gap = Traj/100*(i);
    Step=[Step; Start+gap];
end

ek = mvnrnd(mu,sigma,Num_k);
% ek2 = mvnrnd(mu,sigma,Num_k).*5;
% ek3 = mvnrnd(mu,sigma,Num_k).*5;
% ek4 = mvnrnd(mu,sigma,Num_k).*5;
% ek5 = mvnrnd(mu,sigma,Num_k).*5;

for j = 1 : Num_k
%     Traj_N{j}=[Start; [Step(:,1)+ek(j,:)' Step(:,2)+ek2(j,:)' Step(:,3)+ek3(j,:)' Step(:,4)+ek4(j,:)' Step(:,5)+ek5(j,:)']; End];
    Traj_N{j}=[Start; Step+ek(j,:)'; End];
end

ek = [zeros(Num_k, 1) ek zeros(Num_k, 1)];

end