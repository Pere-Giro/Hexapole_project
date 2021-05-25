%% Secció 1 - Paràmetres geomètrics i constants globals (definicion l negativa)


m.B=1e-3*[0,-89.15,0;0,-89.15,0;77.21,44.57,0;77.21,44.57,0;-77.21,44.57,0;-77.21,44.57,0]';
m.A=1e-3*[-231.62,-136.18,0;231.62,-136.18,0;233.74,-132.5,0;2.13,268.67,0;-2.13,268.67,0;-233.74,-132.5,0]';

m.lc=-1*[0.381915756810321;0.381915756810321;0.381910232646364;0.381913362426611;0.381913362426611;0.381910232646364];%lengths of the cables in home position
m.ro=0.025/2; %radius of the pulley fixed in MX-28 load axis [m]

%Data that was tested first try units [mm]

% B=[0,-89.15,0;0,-89.15,0;77.21,44.57,0;77.21,44.57,0;-77.21,44.57,0;-77.21,44.57,0]';
% 
% A=[-231.62,-136.18,0;231.62,-136.18,0;233.74,-132.5,0;2.13,268.67,0;-2.13,268.67,0;-233.74,-132.5,0]'; 
% 
% ls0000=[309.612088426793;309.612088426793;309.605274179882;309.609134878156;309.609134878156;309.605274179882];
% p0000=[0 0 -200];
% rpy0000=[0 0 0];
% p1=[3 3 203];
% rpy1=[0.0349065850398866 0.0349065850398866 0.0349065850398866];


%new data units  in meters [m]
% ls0000=[0.381915756810321;0.381915756810321;0.381910232646364;0.381913362426611;0.381913362426611;0.381910232646364];

% old home position
ls0000=-1e-3*[309.612088426793;309.612088426793;309.605274179882;309.609134878156;309.609134878156;309.605274179882];

p0000= [0 0 -0.3]';
rpy0000= [0 0 0]';
p1=[0.003 0.003 -0.303]';
rpy1=deg2rad(1)*ones(3,1);

xthetastar(1:8)=[0,0,0,0,0,0,0,0]';
xthetastar(9:16)=zeros(8,1);



%% Secció 2 - Mètode de Newton
[pfinal1,rpyfinal1]=FKNewtonMeth(zeros(16,1),p1,rpy1,m); % xtheta,p0,rpy0,m



%% Secció 3 - Provem ara de fer input de theta

xtheta(1:8)=[deg2rad(90)*ones(1,6),zeros(1,2)]'; %[rad]
xtheta(9:16)=zeros(8,1); %[rad/s]
% [pfinal2,rpyfinal2,time2]=FKNewtonMeth(xtheta,p1,rpy1,m,'Analytic' );

[pfinal1,rpyfinal1]=FKfsolve(zeros(6,1),p1,rpy1,m);





%% Jacobian Trial For some reason the sign of Analitic jacobian is not correct it shouls be the same as FinDif
JJ1=FKJacobianFinDif(ls0000,p1,rpy1,m.A);
JJ2=FKJacobianAnalitic(p1,rpy1);



%% Proves amb cable lenghts
p1=[0.003 0.003 -0.303]';
rpy1=deg2rad(1)*ones(3,1);
xtheta=zeros(6,1);
[pff,rpyff]=FKfsolve(xtheta,p1,rpy1,m);
q.x=pff(1);
q.y=pff(2);
q.z=pff(3);
q.psi=rpyff(1);
q.theta=rpyff(2);
q.phi=rpyff(3);
q.beta1=0;
q.beta2=0;
Draw_DM_MXcord( m, q )

for i=1:20
    pause(1/20);
    xtheta=xtheta+deg2rad(20)*ones(6,1);
    [pff,rpyff]=FKfsolve(xtheta,p1,rpy1,m);
    q.x=pff(1);
    q.y=pff(2);
    q.z=pff(3);
    q.psi=rpyff(1);
    q.theta=rpyff(2);
    q.phi=rpyff(3);
    q.beta1=0;
    q.beta2=0;
    Draw_DM_MXcord( m, q )
    
end

    














