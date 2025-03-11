function zdot=floatingbase(t,z,foot,tal1,tal2,FG1,FG2,FS1,FS2)   

M=foot.M;       m1=foot.m1;   m2=foot.m2;
N=foot.N;       l1=foot.l1;   l2=foot.l2; 
I_N=foot.I_N;   I1=foot.I1;   I2=foot.I2;
g=foot.g;
com_height=foot.com_height;
com_horizen=foot.com_horizen;
x = z(1);y = z(2);phi = z(3);theta1 = z(4);theta2 = z(5);
dx = z(6);dy = z(7);dphi = z(8); dtheta1 = z(9);dtheta2 = z(10);
% slip.tal1=controller(1);
% slip.f1=controller(2);
% slip.tal2=controller(3);
% slip.f2=controller(4);
%         
% tal1=0;
% tal2=0;
% f1=0;
% f2=0;

Fg1x=FG1(1);Fg1y=FG1(2);
Fg2x=FG2(1);Fg2y=FG2(2);

Fs1x=FS1(1);Fs1y=FS1(2);
Fs2x=FS2(1);Fs2y=FS2(2);

AA=[                                                                                                                                        M + m1 + m2,                                                                                                                                                  0,         (l1*m1*sin(phi + theta1))/2 - M*com_height*cos(phi) - (l2*m2*sin(phi - theta2))/2 - M*com_horizen*sin(phi) - (N*m1*sin(phi))/2 + (N*m2*sin(phi))/2,                (l1*m1*sin(phi + theta1))/2,                (l2*m2*sin(phi - theta2))/2;
    0,                                                                                                                                        M + m1 + m2,         (l2*m2*cos(phi - theta2))/2 - (l1*m1*cos(phi + theta1))/2 + M*com_horizen*cos(phi) + (N*m1*cos(phi))/2 - (N*m2*cos(phi))/2 - M*com_height*sin(phi),               -(l1*m1*cos(phi + theta1))/2,               -(l2*m2*cos(phi - theta2))/2;
    (l1*m1*sin(phi + theta1))/2 - M*com_height*cos(phi) - (l2*m2*sin(phi - theta2))/2 - M*com_horizen*sin(phi) - (N*m1*sin(phi))/2 + (N*m2*sin(phi))/2, (l2*m2*cos(phi - theta2))/2 - (l1*m1*cos(phi + theta1))/2 + M*com_horizen*cos(phi) + (N*m1*cos(phi))/2 - (N*m2*cos(phi))/2 - M*com_height*sin(phi), I1 + I2 + I_N + M*com_height^2 + M*com_horizen^2 + (N^2*m1)/4 + (N^2*m2)/4 + (l1^2*m1)/4 + (l2^2*m2)/4 - (N*l1*m1*cos(theta1))/2 - (N*l2*m2*cos(theta2))/2, (m1*l1^2)/4 - (N*m1*cos(theta1)*l1)/4 + I1, (N*l2*m2*cos(theta2))/4 - (l2^2*m2)/4 - I2;
    (l1*m1*sin(phi + theta1))/2,                                                                                                                       -(l1*m1*cos(phi + theta1))/2,                                                                                                                 (m1*l1^2)/4 - (N*m1*cos(theta1)*l1)/4 + I1,                           (m1*l1^2)/4 + I1,                                          0;
    (l2*m2*sin(phi - theta2))/2,                                                                                                                       -(l2*m2*cos(phi - theta2))/2,                                                                                                                 (N*l2*m2*cos(theta2))/4 - (l2^2*m2)/4 - I2,                                          0,                           (m2*l2^2)/4 + I2];
 
RHS=[                                                                                                                                                                                                                                                                                                                                           Fg1x + Fg2x + Fs1x + Fs2x + (dphi^2*l2*m2*cos(phi - theta2))/2 + (dtheta2^2*l2*m2*cos(phi - theta2))/2 - (dphi^2*l1*m1*cos(phi + theta1))/2 - (dtheta1^2*l1*m1*cos(phi + theta1))/2 + M*com_horizen*dphi^2*cos(phi) + (N*dphi^2*m1*cos(phi))/2 - (N*dphi^2*m2*cos(phi))/2 - M*com_height*dphi^2*sin(phi) - dphi*dtheta2*l2*m2*cos(phi - theta2) - dphi*dtheta1*l1*m1*cos(phi + theta1);
                                                                                                                                                                                                                                                                                                                           Fg1y + Fg2y + Fs1y + Fs2y - M*g - g*m1 - g*m2 + (dphi^2*l2*m2*sin(phi - theta2))/2 + (dtheta2^2*l2*m2*sin(phi - theta2))/2 + M*com_height*dphi^2*cos(phi) - (dphi^2*l1*m1*sin(phi + theta1))/2 - (dtheta1^2*l1*m1*sin(phi + theta1))/2 + M*com_horizen*dphi^2*sin(phi) + (N*dphi^2*m1*sin(phi))/2 - (N*dphi^2*m2*sin(phi))/2 - dphi*dtheta2*l2*m2*sin(phi - theta2) - dphi*dtheta1*l1*m1*sin(phi + theta1);
 (N*l2*m2*sin(theta2)*dtheta2^2)/4 - (N*dphi*l2*m2*sin(theta2)*dtheta2)/2 + Fg2y*l2*cos(phi - theta2) + Fs2y*l2*cos(phi - theta2) - Fg2x*l2*sin(phi - theta2) - Fs2x*l2*sin(phi - theta2) - Fg1y*l1*cos(phi + theta1) - Fs1y*l1*cos(phi + theta1) + (Fg1y*N*cos(phi))/2 - (Fg2y*N*cos(phi))/2 + (Fs1y*N*cos(phi))/2 - (Fs2y*N*cos(phi))/2 + Fg1x*l1*sin(phi + theta1) + Fs1x*l1*sin(phi + theta1) - (Fg1x*N*sin(phi))/2 + (Fg2x*N*sin(phi))/2 - (Fs1x*N*sin(phi))/2 + (Fs2x*N*sin(phi))/2 + (g*l1*m1*cos(phi + theta1))/2 - M*com_horizen*g*cos(phi) - (N*g*m1*cos(phi))/2 + (N*g*m2*cos(phi))/2 + M*com_height*g*sin(phi) - (g*l2*m2*cos(phi - theta2))/2 - (N*dtheta1^2*l1*m1*sin(theta1))/4 - (N*dphi*dtheta1*l1*m1*sin(theta1))/2;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                (N*l1*m1*sin(theta1)*dphi^2)/4 + tal1 - Fg1y*l1*cos(phi + theta1) - Fs1y*l1*cos(phi + theta1) + Fg1x*l1*sin(phi + theta1) + Fs1x*l1*sin(phi + theta1) + (g*l1*m1*cos(phi + theta1))/2;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                (N*l2*m2*sin(theta2)*dphi^2)/4 + tal2 - Fg2y*l2*cos(phi - theta2) - Fs2y*l2*cos(phi - theta2) + Fg2x*l2*sin(phi - theta2) + Fs2x*l2*sin(phi - theta2) + (g*l2*m2*cos(phi - theta2))/2];
 
qdd= AA \ RHS;

zdot=[dx,dy,dphi,dtheta1,dtheta2,...
      qdd(1) qdd(2) qdd(3) qdd(4) qdd(5)]';

