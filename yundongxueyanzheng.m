clear functions %
clear all

foot.m1   =  1;    
foot.m2   =  1;  % masses
foot.I1   =  0.5;  
foot.I2   =  0.5;  % inertias about cms
foot.l1   =  2;              % length of links
foot.l2   =  1;   
foot.N   =  1;  
foot.a1   =  0.5;            % dist. from O to G1 and E to G2 (see figures)
foot.a2   =  0.5;   
foot.g    =  10;

framespersec=50;  %if view is not speeded or slowed in dbpend_animate
T=5;             %duration of animation  in seconds
tspan=linspace(0,T,T*framespersec);
x=0;
dx=0;
y=1;
dy=0;
phi=0;
dphi=0;


q1    = pi*7/8; %angle made by link1 with vertical
dq1    = 0;        %abslolute velocity of link1   
q2    = pi*2/3 ;      %angle made by link2 with vertical
dq2    = 0;        %abslolute velocity of link2

z0=[x,y,phi,q1,q2,dx,dy,dphi,dq1,dq2]';
 [PA,PB,PC,PD,PT,PO]=kinematic(z0,i,foot);


figure(1)

    plot(PO(1),PO(2),'ko','MarkerSize',8); %pivot point
    hold on
    line([PT(1) PO(1)],[PT(2) PO(2)],'Linewidth',4,'Color',[0.8 0 0]);% first pendulum
    line([PA(1) PB(1)],[PA(2) PB(2)],'Linewidth',4,'Color',[0 0.8 0]);% second pendulum
    line([PA(1) PC(1)],[PA(2) PC(2)],'Linewidth',4,'Color',[0 0 0]);% second pendulum
    line([PB(1) PD(1)],[PB(2) PD(2)],'Linewidth',4,'Color',[0 0 0]);% second pendulum
    
    axis([-2*foot.l1 2*foot.l1 -2*foot.l1 2*foot.l1]);
    axis square
    hold off