%% Secci� 1 - Par�metres geom�trics i constants globals

m.B=1e-3*[0,-89.15,0;0,-89.15,0;77.21,44.57,0;77.21,44.57,0;-77.21,44.57,0;-77.21,44.57,0]';
m.A=1e-3*[-231.62,-136.18,0;231.62,-136.18,0;233.74,-132.5,0;2.13,268.67,0;-2.13,268.67,0;-233.74,-132.5,0]';


% m.lc=-1*[0.381915756810321;0.381915756810321;0.381910232646364;0.381913362426611;0.381913362426611;0.381910232646364]; %lengths of the cables in home position for zxstar =-0.3m


m.ro=0.025/2;

xstar(1)=0;
xstar(2)=0;
xstar(3)=-0.300;
xstar(4)=0;
xstar(5)=0;
xstar(6)=0;
xstar(7)=0;
xstar(8)=0;

xstar(9)=0;
xstar(10)=0;
xstar(11)=0;
xstar(12)=0;
xstar(13)=0;
xstar(14)=0;
xstar(15)=0;
xstar(16)=0;

if isrow(xstar)
    xstar=xstar';
else
    xstar=xstar;
end
        

%Data that was tested first try units [mm]
% ls0000=[309.612088426793;309.612088426793;309.605274179882;309.609134878156;309.609134878156;309.605274179882];
% p0000=[0 0 -200];
% rpy0000=[0 0 0];
% p1=[3 3 203];
% rpy1=[0.0349065850398866 0.0349065850398866 0.0349065850398866];

%new data units  in meters [m]
ls0000=-1*[0.381915756810321;0.381915756810321;0.381910232646364;0.381913362426611;0.381913362426611;0.381910232646364];
p0000= [0 0 -0.3]';
rpy0000= [0 0 0]';
x0000=[p0000;rpy0000];


x2=[0,0,-0.3,0,0,0,deg2rad(1),0,0,0,0,0,0,0,0,0]';


%% Secci� 2 - M�tode de Newton
[lsfinal]=IKsolve(x0000,m);
theta6=L_to_theta(IKsolve(xstar,m),m);
theta6_1=L_to_theta(IKsolve(x2,m),m);
