function [  ] = Draw_DM_HexaMX28( m, q )
%DRAW_DM Summary of this function goes here
%   Detailed explanation goes here
p=[q.x,q.y,q.z]';
figure(100);
pos_fig1 = [0 0 1280 720];
set(gcf,'Position',pos_fig1)


l=m.ls;
Bp=m.B;

x(1)=q.x;
x(2)=q.y;
x(3)=q.z;
x(4)=q.psi;
x(5)=q.theta;
x(6)=q.phi;
x(7)=q.beta1;
x(8)=q.beta2;



%biabs == matrix with all the positions vectors that were B matrix, but now
%in ABS ref. (matrix of platform position in ABS ref.) 

%biabs[:,i]= p + m.B[:,i];
biabs = [x(1) + cos(x(5)) * sin(x(6)) * 0.891500000000000070e-1 x(1) + cos(x(5)) * sin(x(6)) * 0.891500000000000070e-1 x(1) + cos(x(5)) * cos(x(6)) * 0.772100000000000009e-1 + cos(x(5)) * sin(x(6)) * (-0.445699999999999985e-1) x(1) + cos(x(5)) * cos(x(6)) * 0.772100000000000009e-1 + cos(x(5)) * sin(x(6)) * (-0.445699999999999985e-1) x(1) + cos(x(5)) * cos(x(6)) * (-0.772100000000000009e-1) + cos(x(5)) * sin(x(6)) * (-0.445699999999999985e-1) x(1) + cos(x(5)) * cos(x(6)) * (-0.772100000000000009e-1) + cos(x(5)) * sin(x(6)) * (-0.445699999999999985e-1); x(2) + sin(x(4)) * sin(x(5)) * sin(x(6)) * 0.891500000000000070e-1 + cos(x(4)) * cos(x(6)) * (-0.891500000000000070e-1) x(2) + sin(x(4)) * sin(x(5)) * sin(x(6)) * 0.891500000000000070e-1 + cos(x(4)) * cos(x(6)) * (-0.891500000000000070e-1) x(2) + sin(x(4)) * sin(x(5)) * cos(x(6)) * 0.772100000000000009e-1 + cos(x(4)) * sin(x(6)) * 0.772100000000000009e-1 + sin(x(4)) * sin(x(5)) * sin(x(6)) * (-0.445699999999999985e-1) + cos(x(4)) * cos(x(6)) * 0.445699999999999985e-1 x(2) + sin(x(4)) * sin(x(5)) * cos(x(6)) * 0.772100000000000009e-1 + cos(x(4)) * sin(x(6)) * 0.772100000000000009e-1 + sin(x(4)) * sin(x(5)) * sin(x(6)) * (-0.445699999999999985e-1) + cos(x(4)) * cos(x(6)) * 0.445699999999999985e-1 x(2) + sin(x(4)) * sin(x(5)) * cos(x(6)) * (-0.772100000000000009e-1) + cos(x(4)) * sin(x(6)) * (-0.772100000000000009e-1) + sin(x(4)) * sin(x(5)) * sin(x(6)) * (-0.445699999999999985e-1) + cos(x(4)) * cos(x(6)) * 0.445699999999999985e-1 x(2) + sin(x(4)) * sin(x(5)) * cos(x(6)) * (-0.772100000000000009e-1) + cos(x(4)) * sin(x(6)) * (-0.772100000000000009e-1) + sin(x(4)) * sin(x(5)) * sin(x(6)) * (-0.445699999999999985e-1) + cos(x(4)) * cos(x(6)) * 0.445699999999999985e-1; x(3) + cos(x(4)) * sin(x(5)) * sin(x(6)) * (-0.891500000000000070e-1) + sin(x(4)) * cos(x(6)) * (-0.891500000000000070e-1) x(3) + cos(x(4)) * sin(x(5)) * sin(x(6)) * (-0.891500000000000070e-1) + sin(x(4)) * cos(x(6)) * (-0.891500000000000070e-1) x(3) + cos(x(4)) * sin(x(5)) * cos(x(6)) * (-0.772100000000000009e-1) + sin(x(4)) * sin(x(6)) * 0.772100000000000009e-1 + cos(x(4)) * sin(x(5)) * sin(x(6)) * 0.445699999999999985e-1 + sin(x(4)) * cos(x(6)) * 0.445699999999999985e-1 x(3) + cos(x(4)) * sin(x(5)) * cos(x(6)) * (-0.772100000000000009e-1) + sin(x(4)) * sin(x(6)) * 0.772100000000000009e-1 + cos(x(4)) * sin(x(5)) * sin(x(6)) * 0.445699999999999985e-1 + sin(x(4)) * cos(x(6)) * 0.445699999999999985e-1 x(3) + cos(x(4)) * sin(x(5)) * cos(x(6)) * 0.772100000000000009e-1 + sin(x(4)) * sin(x(6)) * (-0.772100000000000009e-1) + cos(x(4)) * sin(x(5)) * sin(x(6)) * 0.445699999999999985e-1 + sin(x(4)) * cos(x(6)) * 0.445699999999999985e-1 x(3) + cos(x(4)) * sin(x(5)) * cos(x(6)) * 0.772100000000000009e-1 + sin(x(4)) * sin(x(6)) * (-0.772100000000000009e-1) + cos(x(4)) * sin(x(5)) * sin(x(6)) * 0.445699999999999985e-1 + sin(x(4)) * cos(x(6)) * 0.445699999999999985e-1;];

%drawing axis
trplot(eye(3), 'length', 0.1, 'color', 'k');
hold on;
axis(1e-3*[-400 400 -400 400 -400 500]); % for 1.5 pendulum is 1e-3*[-400 400 -400 400 -400 1200]
                                          % for 1.8 pendulum is 1e-3*[-400 400 -400 400 -400 1500]
                                          % for 0.8 pendulum is 1e-3*[-400 400 -400 400 -400 500]

%drawing the poligons to see ABS and platform
%drawing ABS
fill3(m.A(1,:),m.A(2,:),m.A(3,:),'b');

%drawing the original position of the platform over ABS (fixed platform
%just to have a reference
% fill3(m.B(1,:),m.B(2,:),m.B(3,:),'w');

%drawing the real platform
fill3(biabs(1,:),biabs(2,:),biabs(3,:),'k');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%drawing the cables
%
for i=1:6
    plot3([biabs(1,i),m.A(1,i)],[biabs(2,i),m.A(2,i)],[biabs(3,i),m.A(3,i)],'color','r');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%drawing the pendulum

%creating rs vector relative to platform;

OGb = [x(1) + sin(x(8)) * l; x(2) - sin(x(7)) * cos(x(8)) * l; x(3) + cos(x(7)) * cos(x(8)) * l;];


[xx,yy,zz]=sphere;

sph=surf(xx*0.025+OGb(1),yy*0.025+OGb(2),zz*0.025+OGb(3));
sph.EdgeColor = 'none';
sph.FaceColor = 'g';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%draw the stick
plot3([p(1),OGb(1)],[p(2),OGb(2)],[p(3),OGb(3)],'color','k');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%draw Gtot position
rgtot=0.012;%radius of the point

Gp=p;
Gb=OGb;

Gtot=1/(m.mp+m.mb)*(m.mp*Gp+m.mb*Gb);


sph1=surf(xx*rgtot+Gtot(1),yy*rgtot+Gtot(2),zz*rgtot+Gtot(3));
sph1.EdgeColor = 'none';
sph1.FaceColor = 'r';


hold off;
end

