% Copyright 2016, Ehab Al Khatib, All rights reserved.


clear all
clc
close all
%%
global  b

b = 0.18;
his.dt=[];

% IO_SFL
dt=0.1;
T=360;
k=2;
bref=0.08;
x=0.2;

%%
y=0.2;
theta=0/2;
X=[x;y;theta];

Vref=0.0;        % max 0.5 m/s
Omegaref=0.0;    % max 3 rps

% Reference Trajectory
tperiod = 2*[0 5 10 15 20 ];

points =[0 0 ;1 0  ; 1 1  ; 0 1; 0 0];
x = points(:,1);
y = points(:,2);


t = 0:dt:tperiod(end);
x_des = interp1(tperiod,x,t);
y_des = interp1(tperiod,y,t);
% plot(x_des,y_des,'.',x_des(1),y_des(1),'ro',x_des(end),y_des(end),'go')


his.X=[];
his.ex=[];
his.ey=[];

%%  Graphics
f3=figure;
ax3=axes('parent',f3);
PLOT.RefTrajectory=line('parent',ax3,'XData',[],'YData',[],'LineStyle',':','color','r','LineWidth',2);
PLOT.Robot=line('parent',ax3,'XData',[],'YData',[],'LineStyle','-','color','g','LineWidth',2);
xlabel('x (m)')
ylabel('y (m)')
legend('Reference Trajectory','Acual ')
grid on

%  robot dimensions
A.R_w = 0.3/2; % robot width/2
A.R_l=0.5/2;   % robot length/2
A.a1 = [-A.R_l -A.R_w]';
A.b1 = [A.R_l -A.R_w/2]';
A.b2 = [A.R_l A.R_w/2]';
A.c = [-A.R_l A.R_w]';
A.P = [A.a1 A.b1 A.b2 A.c];
pl=[];

A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotated car
A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; % add offset of car's center

A.P_robot=patch(A.P(1,:),A.P(2,:),'g');
A.P_robot.XData=A.Prot_trasl(1,:)';
A.P_robot.YData=A.Prot_trasl(2,:)';
axis([-0.5 1.5 -0.5 1.5])



%% Main loop


for i=1:length(x_des)
      
    % Simulate WMR 
    X=Move_Robot(X,[Vref,Omegaref],dt);
    
    %     IOSFL Controller
    xb=X(1)+bref*cos(X(3));
    yb=X(2)+bref*sin(X(3));
    
    ex= x_des(i)-xb;
    ey= y_des(i)-yb;
    
    
    Vd_x=k*ex;
    Vd_y=k*ey;
    
    Vref = cos(X(3))*Vd_x + sin(X(3))*Vd_y ;
    Omegaref = (1/bref)*(cos(X(3))*Vd_y-sin(X(3))*Vd_x);
    
    
    % Store the data
   
    his.X=[his.X X];
    his.ex=[his.ex ex];
    his.ey=[his.ey ey];
    
    % update the figures
   
    set(PLOT.RefTrajectory,'XData',x_des(1:i),'YData',y_des(1:i));
    set(PLOT.Robot,'XData',his.X(1,:),'YData',his.X(2,:));
    
    A.Rot = [ cos(X(3)) -sin(X(3)); sin(X(3)) cos(X(3))]*A.P; %rotated car
    A.Prot_trasl = A.Rot + [ ones(1,4)*X(1); ones(1,4)*X(2)]; % add offset of car's center
    A.P_robot.XData=A.Prot_trasl(1,:)';
    A.P_robot.YData=A.Prot_trasl(2,:)';
    drawnow
  
    
end




