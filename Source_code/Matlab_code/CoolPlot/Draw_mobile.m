%% Draw_mobile


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% This function has to be used with Draw_STL, to obtain the video of the
% simulation. In particular, this functions draws de cables and the pole
% given a vector of configuration q

%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% m > structural parameter containing all geometric variables that are used to 
% define the model (such as gravity constant g, or anchor points Ai, Bi)  

% q > configuration vector in a particular instant of time, given in
% platform coordinates


%%%%%%%%%%%%%%%%%%%%%% Outputs %%%%%%%%%%%%%%%%%%%%%%

% plot of the system in a particular configuration

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Array_plot ] = Draw_mobile( m,q )

figure(100);

% all units in [mm]

x(1)=1e3*(q.x);
x(2)=1e3*(q.y);
x(3)=1e3*(q.z);
x(4)=q.psi;
x(5)=q.theta;
x(6)=q.phi;
x(7)=q.beta1;
x(8)=q.beta2;

A=1e3*m.A;
l=1e3*m.ls;

% biabs == matrix with all the positions vectors that were B matrix, but now
% in ABS ref. (matrix of platform position in ABS ref.) 

% biabs[:,i]= p + m.B[:,i]; in [mm]

for i=1:6
	biabs[:,i]= p + R*m.B[:,i];
end

hold on
 
% Drawing the real platform

plat_Draw=fill3(biabs(1,:),biabs(2,:),biabs(3,:),'k');

% Drawing the cables
w1=plot3([biabs(1,1),A(1,1)],[biabs(2,1),A(2,1)],[biabs(3,1),A(3,1)], ... 
                                                             'color','y');
w2=plot3([biabs(1,2),A(1,2)],[biabs(2,2),A(2,2)],[biabs(3,2),A(3,2)], ... 
                                                             'color','y');
w3=plot3([biabs(1,3),A(1,3)],[biabs(2,3),A(2,3)],[biabs(3,3),A(3,3)], ... 
                                                             'color','y');

w4=plot3([biabs(1,4),A(1,4)],[biabs(2,4),A(2,4)],[biabs(3,4),A(3,4)], ... 
                                                             'color','y');
w5=plot3([biabs(1,5),A(1,5)],[biabs(2,5),A(2,5)],[biabs(3,5),A(3,5)], ... 
                                                             'color','y');
w6=plot3([biabs(1,6),A(1,6)],[biabs(2,6),A(2,6)],[biabs(3,6),A(3,6)], ... 
                                                             'color','y');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drawing the pendulum

% Creating rs vector relative to platform;

OGb = [x(1) + sin(x(8)) * l; x(2) - sin(x(7)) * cos(x(8)) * l; x(3) + cos(x(7)) * cos(x(8)) * l;];


[xx,yy,zz]=sphere;

radius=0.025e3; % in [mm]
 
sph=surf(xx*radius+OGb(1),yy*radius+OGb(2),zz*radius+OGb(3));
sph.EdgeColor = 'none';
sph.FaceColor = 'y';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Draw the stick
stick=plot3([x(1),OGb(1)],[x(2),OGb(2)],[x(3),OGb(3)],'color','k');


hold off

Array_plot = [plat_Draw,w1,w2,w3,w4,w5,w6, sph, stick  ];
drawnow;

end