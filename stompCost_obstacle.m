function cost = stompCost_obstacle(robot,radius,Env)

e = 5;
cost = 0;
robot = round(robot);

for i = 1 : length(robot)
    cost = cost + max( e + radius(i) - Env(robot(i,:)),0);
end 

end