function [tal1,tal2,FS1,FS2]=adding_spring(t,z,foot)   

M=foot.M;       m1=foot.m1;   m2=foot.m2;
N=foot.N;       l1=foot.l1;   l2=foot.l2; 
I_N=foot.I_N;   I1=foot.I1;   I2=foot.I2;
g=foot.g;


x = z(1);y = z(2);phi = z(3);theta1 = z(4);theta2 = z(5);
dx = z(6);dy = z(7);dphi = z(8); dtheta1 = z(9);dtheta2 = z(10);

spring_kpA=200;     spring_kpB=200;     spring_kpL=-1000000;
spring_kdA=-1;     spring_kdB=-1;     spring_kdL=-50;

 tal1=(foot.theta10-theta1)*spring_kpA+dtheta1*spring_kdA;
 tal2=(foot.theta20-theta2)*spring_kpB+dtheta2*spring_kdB;

[~,~,PC,PD,~,~]=kinematic(z,M,foot);
L=norm(PC-PD);

spring_velocity = - ((l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi)) - l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)) + N*sin(phi))*(dphi*(l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)) - (N*cos(phi))/2) + dphi*(l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2)) - (N*cos(phi))/2) + dtheta1*l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)) - dtheta2*l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2))))/(abs(l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)) + l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2)) - N*cos(phi))^2 + abs(l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi)) - l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)) + N*sin(phi))^2)^(1/2) - ((l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)) + l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2)) - N*cos(phi))*(dphi*(l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)) - (N*sin(phi))/2) - dphi*(l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi)) + (N*sin(phi))/2) + dtheta1*l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)) + dtheta2*l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi))))/(abs(l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)) + l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2)) - N*cos(phi))^2 + abs(l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi)) - l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)) + N*sin(phi))^2)^(1/2);
F_spring=spring_kpL*(L-foot.L0)*((PC-PD)/L)+spring_kdL*spring_velocity;

 FS1=F_spring;
 FS2=-F_spring;
 
