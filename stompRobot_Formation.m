function [robot, radi] = stompRobot_Formation(X_mat)
robots = cell(3, 2);
% X_mat = round(X_mat);

a1 = 50;
a2 = 40;
a3 = 25;
a4 = 35;

%circle radius for different parts
robots{1,2} = 20;
robots{2,2} = 20;
robots{3,2} = 20;

%sample numbers for different areas
n_p1=40;
n_p2=40;
n_p3=40;

x1 = X_mat(2, 1); x2 = X_mat(3, 1);
y1 = X_mat(2, 2); y2 = X_mat(3, 2);
z1 = X_mat(2, 3); z2 = X_mat(3, 3);
P1 = [(linspace(x1,x2,n_p1))',(linspace(y1,y2,n_p1))',(linspace(z1,z2,n_p1))'];
robots{1,1} = P1;


x1 = X_mat(3, 1); x2 = X_mat(4, 1);
y1 = X_mat(3, 2); y2 = X_mat(4, 2);
z1 = X_mat(3, 3); z2 = X_mat(4, 3);
P2 = [(linspace(x1,x2,n_p2))',(linspace(y1,y2,n_p2))',(linspace(z1,z2,n_p2))'];
robots{2,1} = P2; 


x1 = X_mat(4, 1); x2 = X_mat(6, 1);
y1 = X_mat(4, 2); y2 = X_mat(6, 2);
z1 = X_mat(4, 3); z2 = X_mat(6, 3);
P3 = [(linspace(x1,x2,n_p3))',(linspace(y1,y2,n_p3))',(linspace(z1,z2,n_p3))'];
robots{3,1} = P3;


robot = [robots{1,1}; robots{2,1}; robots{3,1}];
radi = [robots{1,2}; robots{2,2}; robots{3,2}];
end