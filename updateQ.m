% Fill in this function with the mathematics for computing the positions of
% each of the joints of the Lynx robot and of the gripper.
%
% input: q = 1x6 vector [q1, q2, q3, q4, q5, q6] of joint angles (q1-q5)
%            and grip distance (q6)
% output: X = 6x3 matrix of values [x1 y1 z1; x2 y2 z2; ...] containing the
%             positions of each of the joints and the center of the gripper
%             got the given input q
%         T = 4x4x6 matrix of transformation matrices, where each slice
%             T(:,:,i) is the coordinate transformation between links i-1
%             and i
%
% Solution provided by: Nick Kneier (MEAM 520 2017 TA)

function [X, T] = updateQ(q)

T = zeros(4,4,6);
% T = 0;

%Important parameters for the robot
L1 = 3*25.4;          %base height (in mm)
L2 = 5.75*25.4;       %shoulder to elbow length (in mm)
L3 = 7.375*25.4;      %elbow to wrist length (in mm)
L4 = 1.75*25.4;       %Wrist1 to Wrist2 (in mm)
L5 = 1.25*25.4;       %wrist2 to base of gripper (in mm)
L6 = 1.125*25.4;      %gripper length (in mm)
PI = pi();            %PI constant


%Frame 1 w.r.t Frame 0
A1 = [cos(q(1)) -sin(q(1))*cos(-PI/2)  sin(q(1))*sin(-PI/2)  0;
      sin(q(1))  cos(q(1))*cos(-PI/2) -cos(q(1))*sin(-PI/2)  0;
              0            sin(-PI/2)            cos(-PI/2) L1;
              0                     0                  0     1];
          
%Frame 2 w.r.t Frame 1          
A2 = [cos(q(2)-(PI/2)) -sin(q(2)-(PI/2))  0   L2*cos(q(2)-(PI/2));
      sin(q(2)-(PI/2))  cos(q(2)-(PI/2))  0   L2*sin(q(2)-(PI/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 3 w.r.t Frame 2
A3 = [cos(q(3)+(PI/2)) -sin(q(3)+(PI/2))  0   L3*cos(q(3)+(PI/2));
      sin(q(3)+(PI/2))  cos(q(3)+(PI/2))  0   L3*sin(q(3)+(PI/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 4 w.r.t Frame 3
A4 = [cos(q(4)-(PI/2)) -sin(q(4)-(PI/2))*cos(-PI/2)   sin(q(4)-(PI/2))*sin(-PI/2)   0;
      sin(q(4)-(PI/2))  cos(q(4)-(PI/2))*cos(-PI/2)  -cos(q(4)-(PI/2))*sin(-PI/2)   0;
              0                          sin(-PI/2)                    cos(-PI/2)   0;
              0                                   0                             0   1];
%Frame 5 w.r.t Frame 4 
A5 = [cos(q(5)) -sin(q(5))  0        0;
      sin(q(5))  cos(q(5))  0        0;
              0          0  1  L4 + L5;
              0          0  0        1];
          
%Gripper Frame w.r.t. Frame 5
A6 = [1 0 0  0;
      0 1 0  0;
      0 0 1  L6;
      0 0 0  1];

%Puts the Homogeneous Tranformations in T Matrix
T(:,:,1) = A1;
T(:,:,2) = A2;
T(:,:,3) = A3;
T(:,:,4) = A4;
T(:,:,5) = A5;
T(:,:,6) = A6;


%Position of First Joint (Base Revolute)
X(1,:) = [0 0 0 1];
%Position of Second Joint (Shoulder Revolute)
X(2,:) = ((A1)*[0;0;0;1])';
%Position of Third Joint (Elbow Revolute)
X(3,:) = ((A1*A2)*[0;0;0;1])';
%Position of Fourth Joint (1st Wrist)
X(4,:) = ((A1*A2*A3)*[0;0;0;1])';
%Position of Fifth Joint (2nd Wrist)
X(5,:) = ((A1*A2*A3*A4)*[0;0;0;1])';
%Position of Gripper (Middle of the Gripper)
X(6,:) = ((A1*A2*A3*A4*A5)*[0;0;0;1])';

%Outputs the 6x3 of the locations of each joint in the Base Frame
X = X(:,1:3);


end