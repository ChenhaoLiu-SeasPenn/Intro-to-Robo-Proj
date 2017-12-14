function [Traj_N,ek]=stompCompute_NoisyTraj(n_Samples,Start,End,R, theta)
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

ek = [zeros(Num_k, 1) ek zeros(Num_k, 1)];

for j = 1 : Num_k
    Traj_N{j}= theta+[ek(j,:);ek(j,:);ek(j,:);ek(j,:);ek(j,:)];
%     Traj_N{j}=[Start; Step+ek(j,:)'; End];
end



end