function [ xthetadot,xq ] = xdot_HexaMX28_Simulink( xtheta, uv, xq) %original input [t, x, u_handle, ustar, xstar, K, uo_handle]
%XDOT_HEXAPOLE Summary of this function goes here     %input to run with Matlab (x,u_handle,t) //to run with simulink (x,u)  
%   differential equation (non-linearized), that modelises our hexapole
%   system (special verion modified to be used with simulink)


%Warning defining data!!

ro=0.025/2; %radius of the pulley fixed in MX-28 load axis [m]
a=(193*0.836*0.0107)/8.3; %constant to be used to indroduce MX-28 to DM
b=-((193*0.836)*(0.0107)^2*193)/8.3-(193*0.836)*8.87e-8*193; %constant to be used to indroduce MX-28 to DM
c=-(193*0.836)*193*8.68e-8; %constant to be used to indroduce MX-28 to DM

%A and B matrices of the hexapole geometry, all distances in [m]

Bgeom=1e-3*[0,-89.15,0;0,-89.15,0;77.21,44.57,0;77.21,44.57,0;-77.21,44.57,0;-77.21,44.57,0]';


Ageom=1e-3*[-231.62,-136.18,0;231.62,-136.18,0;233.74,-132.5,0;2.13,268.67,0;-2.13,268.67,0;-233.74,-132.5,0]';


lc=-1*[0.381915756810321;0.381915756810321;0.381910232646364;0.381913362426611;0.381913362426611;0.381910232646364];%lengths of the cables in home position


p0=xq(1:3,1);
rpy0=xq(4:6,1);

[ pf,rpyf ] = FKNewtonSimulink( xtheta,p0,rpy0,lc,ro,Ageom,Bgeom );

x=zeros(16,1);

x(1:8,1)=[pf;rpyf;xtheta(7:8,1)];

H=Hmatrix(x);
G=Gmatrix(x);
E=Ematrix(x);
Ji=Jimatrix(x);

x(9:16,1)=Ji\xtheta(9:16,1);

xq=x;

COR=CORmatrix(x);
Jidot=JidotmatrixSimulink(x,ro);


invJi=pinv(Ji);


%Now we build needed matrices for HexaMX_28 model
Hm=[c*(1/ro)*E,zeros(8,2)];
Cm=[b*(1/ro)*E,zeros(8,2)];

Mthetam=(H*invJi-Hm);
Cthetam=((COR-H*invJi*Jidot)*invJi-Cm);


thetadot=xtheta(9:16);
thetadotdot=Mthetam\((1/ro)*E*a*(uv)-Cthetam*thetadot-G);
xthetadot=[thetadot; thetadotdot];
 


end

