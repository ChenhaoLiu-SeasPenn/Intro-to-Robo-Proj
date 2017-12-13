function cost = stompCost_obstacle(robot,radius,Env)

e = 5;
cost = 0;
robot = round(robot);

idx = round(robot/10) + [10, 60, 10];
cost = max(e + radius - Env(sub2ind([120, 60, 70], idx(2, :), idx(1, :), idx(3, :))),0);
cost = sum(sum(cost));

% for i = 1 : length(robot)
%     idx = round(robot(i,:)/10) + [10, 60, 10];
% %     disp(robot(i,:));
% %     disp(idx);
%     if (idx(1)<0) || (idx(2)<0) || (idx(3)<0)
%         disp(idx);
%     end
%     cost = cost + max( e + radius(i) - Env(idx(2), idx(1), idx(3)),0);
% end 

end