function [t_ode,z_ode] = floatingbaseode(tt,zz,foot)
t0=tt;
z0=zz;
t_ode=t0;
z_ode=z0;
d_ground=-1000;
k_ground=-200000;
d_groundx=-1000;
dt=0.001;
Nn=10000;
M=foot.M;       m1=foot.m1;   m2=foot.m2;
N=foot.N;       l1=foot.l1;   l2=foot.l2; 
I_N=foot.I_N;   I1=foot.I1;   I2=foot.I2;
g=foot.g;
for i=1:9000
    
    %[x,y,leg_theta_left,leg_length_left,leg_theta_right,leg_length_right,dx,dy,dleg_theta_left,dleg_length_left,dleg_theta_right,dleg_length_right]=z0(1:end);
    x = z0(1);
    y = z0(2);
    phi = z0(3);
    theta1 = z0(4);
    theta2 = z0(5);
    dx = z0(6);
    dy = z0(7);
    dphi = z0(8);
    dtheta1 = z0(9);
    dtheta2 = z0(10);
    [PA,PB,PC,PD,PT,PO,PG]=kinematic(z0,i,foot);
    nnn=(PC(2)>=0)*2+(PD(2)>=0);
    
    switch nnn
        case 0
            FG1=[d_groundx*(dx + dphi*(l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)) - (N*sin(phi))/2) + dtheta1*l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)));
                k_ground*PC(2)+...
                d_ground*( dy - dphi*(l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)) - (N*cos(phi))/2) - dtheta1*l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)))];
            FG2=[d_groundx*(dx + dphi*(l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi)) + (N*sin(phi))/2) - dtheta2*l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi)));
                k_ground*PD(2)+...
                d_ground*(dy + dphi*(l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2)) - (N*cos(phi))/2) - dtheta2*l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2)))];
            %%%%地面不会给负向力
            if FG1(2)<0
                FG1(2)=0;
            end
            if FG2(2)<0
                FG2(2)=0;
            end
        case 1
            FG1=[d_groundx*(dx + dphi*(l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)) - (N*sin(phi))/2) + dtheta1*l1*(cos(phi)*sin(theta1) + cos(theta1)*sin(phi)));
                k_ground*PC(2)+...
                d_ground*(dy - dphi*(l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)) - (N*cos(phi))/2) - dtheta1*l1*(cos(phi)*cos(theta1) - sin(phi)*sin(theta1)))];
            FG2=zeros(2,1);
            if FG1(2)<0
                FG1(2)=0;
            end
            if FG2(2)<0
                FG2(2)=0;
            end
        case 2
            FG1=zeros(2,1);
            FG2=[d_groundx*(dx + dphi*(l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi)) + (N*sin(phi))/2) - dtheta2*l2*(cos(phi)*sin(theta2) - cos(theta2)*sin(phi)));
                k_ground*PD(2)+...
                d_ground*(dy + dphi*(l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2)) - (N*cos(phi))/2) - dtheta2*l2*(cos(phi)*cos(theta2) + sin(phi)*sin(theta2)))];
            if FG1(2)<0
                FG1(2)=0;
            end
            if FG2(2)<0
                FG2(2)=0;
            end
        case 3
            FG1=zeros(2,1);
            FG2=zeros(2,1);
    end
    
    [tal1,tal2,FS1,FS2] = adding_spring(t0,z0,foot);
    options1 = odeset('Abstol',1e-13,'Reltol',1e-13);
    tspan = linspace(t0,t0+dt,dt*Nn);
    [t_temp z_temp] = ode113(@floatingbase,tspan,z0,options1,foot,tal1,tal2,FG1,FG2,FS1,FS2);
    
    t0 = t_temp(end,:);
    z0 = z_temp(end,:);
    t_ode = [t_ode;t_temp(2:end)];
    z_ode = [z_ode;z_temp(2:end,:)];

    if (PT(2)<=0)||(PG(2)<=0)
        break;
    end
    if i==3000
        z0(8)=0.8;
    end
end
