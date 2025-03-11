close all; clc;clear;

syms x y phi theta1 theta2 'real' % joint angle, angular rate and acceleration
syms dx dy dphi dtheta1 dtheta2 'real'             % CoM distance relative to parent joint and link length
syms ddx ddy ddphi ddtheta1 ddtheta2 'real'
syms qdd 'real'
syms l1 N l2 'real'
syms m1 M m2 'real'     % mass of links and motors
syms I1 I2 I_N 'real'     % inertia moment of links and motors
syms g 'real'                       % gravatical acceleration
syms tal1 tal2 f1 f2 'real'   
syms Fg1x Fg1y Fg2x Fg2y Fs1x Fs1y Fs2x Fs2y 'real'
syms trunklength 'real' 
syms com_height com_horizen 'real' 
%%%注意两杆的质心位置，即transformation
%%%matrix还有动能势能的表达写法，
%%%�?��要的是在自己定制的坐标系下，角度与角速度要统�?��
%%%就是自己定制坐标系肯定存在一个特定的旋转矩阵，后面相应的角�?度也应该跟随这一矩阵而调�?
%%%主函数中，验证正确�?时，用到的能量图像，也应该跟随这样的旋转矩阵而产生相应的调整�?
%%%总结�?��就是，先�?��旋转矩阵、角速度等，derive后不仅更新mm与ff还要更新ke、kp，最终才能完整验证�?
%%%20211111 zjt

% unit vectors
i = [1; 0; 0];  j = [0; 1; 0];  k = [0; 0; 1];
% variables vectorization
q = [x,y,phi,theta1,theta2]';
qd = [dx,dy,dphi,dtheta1,dtheta2]';
qdd = [ddx,ddy,ddphi,ddtheta1,ddtheta2]';
%qdd = [ddx;ddy;ddleg_theta_left;ddleg_length_left;ddleg_theta_right;ddleg_length_right];
G = -g * j; % gravity vector




%%%%运功�?
R_N=[cos(phi), -sin(phi);
     sin(phi),  cos(phi)]; 

R1=[cos(theta1-pi), -sin(theta1-pi);
    sin(theta1-pi),  cos(theta1-pi)];

R2=[cos(-theta2), -sin(-theta2);
    sin(-theta2),  cos(-theta2)];

                 
                 
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


P=[x,y,0,0,0,0];


%%%%末端位置，P1左脚，P2右脚，P3上身顶端
PA=TN*[N/2;0;1];
PB=TN*[-N/2;0;1];
PC=TN*T1*[l1;0;1];
PD=TN*T2*[l2;0;1];

PO=TN*[0;0;1];
PT=TN*[0;com_height;1];

%%%%重心位置，PG1左脚，PG2右脚，PG3上身重心
PG1=TN*T1*[l1/2;0;1];   
PG2=TN*T2*[l2/2;0;1];
%PGO=TN*[0;0;1]; 
PGO=TN*[com_horizen;com_height;1]; 
%%%%雅可�?
 J1=jacobian(PC(1:2),q);
 J2=jacobian(PD(1:2),q);
% J3=jacobian(P3(1:3),q);
% V1 = jacobian(P1,q)*qd;       % 左腿重心速度
% V2 = jacobian(P2,q)*qd;       % 右腿重心速度
% V3 = jacobian(PGtrunk,q)*qd;
% 
J1dot=[(jacobian(J1(1,:),q)*qd)';
       (jacobian(J1(2,:),q)*qd)'];
J2dot=[(jacobian(J2(1,:),q)*qd)';
       (jacobian(J2(2,:),q)*qd)'];

V_1 = jacobian(PG1(1:2),q)*qd;       % 左腿重心速度
V_2 = jacobian(PG2(1:2),q)*qd;       % 右腿重心速度
V_O = jacobian(PGO(1:2),q)*qd;       %上身重心速度

V_p1 = jacobian(PC(1:2),q)*qd       % 左腿重心速度
V_p2 = jacobian(PD(1:2),q)*qd       % 右腿重心速度
%DL=dot((V_p2-V_p1),((PD(1:2)-PC(1:2))/norm(PD(1:2)-PC(1:2))));
% V_m1 = derivative(P_m1);       % motor 1 velocity(point mass)
% V_m2 = derivative(P_m2);       % motor 2 velocity(point mass)
% angular velocity
W_1 = (dphi+dtheta1);                % link  1 angular velocity
W_2 = (dphi-dtheta2);        % link  2 angular velocity
W_O = dphi;        % link  2 angular velocity

% W_m1 = k_r1 * dq1 * k;         % motor 1 angular velocity
% W_m2 = (dq1 + k_r2 * dq2) * k; % motor 2 angular velocity

% inertia momentum w.r.t. inertial frame(a.k.a. world frame)
% diag_inertia = @(input)( diag([0, 0, input]) ); % convert into diagonal inertia matrix
% I_l1_w = R1 * I1 * R1';       % link   1 inertia
% I_l2_w = R2 * I2 * R2';       % link   2 inertia
% I_N_w = [R_N,0;0,0,1] * I_N * [R_N,0;0,0,1]';       % link   2 inertia

I_l1_w = I1;       % link   1 inertia
I_l2_w = I2;       % link   2 inertia
I_N_w  = I_N;       % link   2 inertia

% I_m1_w = R_m1 * diag_inertia(I_m1) * R_m1';   % motor  1 inertia
% I_m2_w = R_m2 * diag_inertia(I_m2) * R_m2';   % motor  2 inertia

B=[zeros(3,2);
   eye(2)];
u=[tal1,tal2]';

%%%ground reaction force
FG1=[Fg1x,Fg1y]';
FG2=[Fg2x,Fg2y]';

%%%linear spring force
FS1=[Fs1x,Fs1y]';
FS2=[Fs2x,Fs2y]';
F1=FG1+FS1;
F2=FG2+FS2;

% kinematic energy
KE = 0.5 * M * dot(V_O(1:2),V_O(1:2)) + 0.5 * m1 * dot(V_1(1:2),V_1(1:2)) + 0.5 * m2 * dot(V_2(1:2), V_2(1:2)) +...
     0.5 * W_O * I_N_w * W_O + 0.5 * W_1 * I_l1_w * W_1 + 0.5 * W_2 * I_l2_w * W_2;

% potential energy
pg1=PG1(1:2);
pg2=PG2(1:2);
PE = m1 * g * PG1(2) + m2 * g * PG2(2) + M * g * PGO(2);

%% Lagrangian derivation
L = simplify(KE - PE);
DL_Ddq = jacobian(L,qd);
% dDL_Ddq_dt - DL_Dq' = Tau
DL_Dq = jacobian(L,q);

dDL_Ddq_dt = jacobian(DL_Ddq, q) * qd + jacobian(DL_Ddq, qd) *qdd;


eqn = simplify(dDL_Ddq_dt - DL_Dq'- B * u - J1'* F1 - J2' * F2);
%eqn = simplify(dDL_Ddq_dt - DL_Dq');
[MM, FF] = equationsToMatrix(eqn, qdd); % convert equation to : MM * ddq = FF
MM;
FF;
J1;
J2;
AAAA=[MM,        J1',        J2';
      J1,  zeros(2,2),  zeros(2,2);
      J2,  zeros(2,2),  zeros(2,2)];
BBBB=[         FF;
       -J1dot*qd;
       -J2dot*qd];
AAAA
BBBB
  