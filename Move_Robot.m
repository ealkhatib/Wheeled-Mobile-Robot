function [ Xnew ] = Move_Robot( X,u,dt )

x=X(1);
y=X(2);
theta=X(3);
V=u(1);
Omega=u(2);


x=x+dt*V*cos(theta);
y=y+dt*V*sin(theta);
theta=theta+dt*Omega;

Xnew=[x;y;theta];


end

