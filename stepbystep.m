clear functions %
clear all

%%%%%%%%% INITIALIZE PARAMETERS %%%%%%
%Mechanical parameters.

foot.M   =  20;
foot.m1  =  1;  
foot.m2  =  1;  % masses
foot.I1   =  1;  
foot.I2   =  0.5;  % inertias about cms
foot.I_N   =  1;  
foot.l1   =  0.08931;              % length of links
foot.l2   =  0.05;   
foot.N   =  0.05;    
foot.g    =  10;
foot.com_height    =  0.3;
foot.com_horizen   = 0.00;

foot.theta10=pi*158/180;
foot.theta20=pi*138/180;
foot.L0=0.16997;

foot.tal1=0;
foot.tal2=0;

foot.FS1=[0,0];
foot.FS2=[0,0];
foot.FG1=[0,0];
foot.FG2=[0,0];

foot.setstep=5;
% Initial conditions and other settings.
framespersec=1000;  %if view is not speeded or slowed in dbpend_animate
T=5;             %duration of animation  in seconds

x=0;
dx=0;
y=0.2;
dy=0;
phi=0;
dphi=0;
q1    = pi*158/180; %angle made by link1 with vertical
dq1    = 0;        %abslolute velocity of link1   
q2    = pi*138/180 ;      %angle made by link2 with vertical
dq2    = 0;        %abslolute velocity of link2

z0=[x,y,phi,q1,q2,dx,dy,dphi,dq1,dq2];
t0=0;


% dt=0.001;
step=0;
fps = 40;
% N=10000;

%%%%%%% INTEGRATOR or ODE SOLVER %%%%%%%

[t_ode,z_ode]=floatingbaseode(t0,z0,foot);
[t,z] = loco_interpolate(t_ode,z_ode,fps);
%%%%%%% POSTPROCESSING %%%%%%%%%%%%%%%%
% A routine to animate the results
% To speed up change the framespersecond
figure(1)
for i=1:length(t)
    
    window_xmin = -0.4*1; window_xmax = 0.4*1;
    window_ymin = -0.2*1; window_ymax = 0.4*1;
    [PA,PB,PC,PD,PT,PO,PGO]=kinematic(z(i,:),i,foot);
    plot(PO(1),PO(2),'ko','MarkerSize',8); %pivot point
    plot(PGO(1),PGO(2),'bo','MarkerSize',8);
    line([PT(1) PGO(1)],[PT(2) PGO(2)],'Linewidth',2,'Color',[0.8 0 0]);
    line([PT(1) PO(1)],[PT(2) PO(2)],'Linewidth',2,'Color',[0.8 0 0]);% first pendulum
    line([PA(1) PB(1)],[PA(2) PB(2)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
    line([PA(1) PC(1)],[PA(2) PC(2)],'Linewidth',4,'Color',[0 0 0]);% second pendulum
    line([PB(1) PD(1)],[PB(2) PD(2)],'Linewidth',4,'Color',[0 0 0]);% second pendulum
    line([PC(1) PD(1)],[PC(2) PD(2)],'Linewidth',2.5,'Color',[0.8 0.8 0.8]);% second pendulum
    line([-5 5],[0 0],'Linewidth',1,'Color',[0 0 0]);% second pendulum
    axis('equal')
    axis on
    axis([window_xmin window_xmax window_ymin window_ymax])
    F(i)=getframe;
end
v = VideoWriter('ccc.avi');
open(v);
writeVideo(v,F);
close(v);
