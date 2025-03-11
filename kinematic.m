function [P_A,P_B,P_C,P_D,P_T,P_O,P_GO]=kinematic(z,t,foot)
x = z(1);
y = z(2);
phi = z(3);
theta1 = z(4);
theta2 = z(5);
dx = z(6);
dy = z(7);
dphi = z(8); 
dtheta1 = z(9);
dtheta2 = z(10);

N=foot.N;
l1=foot.l1;
l2=foot.l2;

com_height=foot.com_height;
com_horizen=foot.com_horizen;
% i = [1; 0; 0];  j = [0; 1; 0];  k = [0; 0; 1];
% q = [q1; q2];
% dq = [qd1; qd2];
% 
% P_E1 = l1 * sin(q1) * i + l1 * -cos(q1) * j; % position of E1
% P_E2 = (l1 * sin(q1) + l2 * sin(q1 - q2)) * i + (l1 * -cos(q1) + l2 * -cos(q1 - q2)) * j; % position of E2
R_N=[cos(phi), -sin(phi);
     sin(phi),  cos(phi)]; %%%基座旋转矩阵

R1=[cos(theta1-pi), -sin(theta1-pi);
    sin(theta1-pi),  cos(theta1-pi)];%%%左脚旋转矩阵

R2=[cos(-theta2), -sin(-theta2);
    sin(-theta2),  cos(-theta2)];  %%%右脚旋转矩阵

qicijuzhen=[0,0,1];
O=[x;y];
O1=[N/2;0];
O2=[-N/2;0];

TN=[R_N,     O;
           qicijuzhen];%%%基座的转换矩�?
T1=[R1,     O1;
    qicijuzhen];%%%基座到左腿末端的转换矩阵
T2=[R2,     O2;
    qicijuzhen];%%%基座到右腿末端的转换矩阵

%%%%末端位置，P1左脚，P2右脚，P3上身顶端
P_A=TN*[N/2;0;1];
P_B=TN*[-N/2;0;1];
P_C=TN*T1*[l1;0;1];
P_D=TN*T2*[l2;0;1];
P_T=TN*[0;com_height;1];
P_O=TN*[0;0;1]; 
P_GO=TN*[com_horizen;com_height;1]; 