function [ xthetadot ] = xdot_HexaMX28_Sim( t, xtheta, m, uvstar, xthetastar, K, uo_handle)
%XDOT_HEXAPOLE Summary of this function goes here
%   differential equation (non-linearized), that modelises our hexapole
%   system

% global xthetaplot
% xthetaplot=horzcat(xthetaplot,xtheta);

 x0=[0,0,-0.3,0,0,0,deg2rad(0),0,0,0,0,0,0,0,0,0]';
        
xq=x0;
         
%global xdotArray

%global Flag3
Flag3= 'No';
% %vector of times necesary to later monitorize cable tensions
switch Flag3
    case 'YES'
        global tvec
        tvec=horzcat(tvec,t);
end

p0=xq(1:3);
rpy0=xq(4:6);

[ pf,rpyf ] = FKNewtonMeth( xtheta,p0,rpy0,m );



x(1:8,1)=[pf;rpyf;xtheta(7:8)];

H=Hmatrix(x);
G=Gmatrix(x);
E=Ematrix(x);
Ji=Jimatrix(x);

x(9:16,1)=Ji\xtheta(9:16);

xq=x;

COR=CORmatrix(x);
Jidot=Jidotmatrix(x,m);


invJi=pinv(Ji);

%Now we build needed matrices for HexaMX_28 model
Hm=[m.c*(1/m.ro)*E,zeros(8,2)];
Cm=[m.b*(1/m.ro)*E,zeros(8,2)];

Mthetam=(H*invJi-Hm);
Cthetam=((COR-H*invJi*Jidot)*invJi-Cm);


        uv=u_function_HexaMX28(t,xtheta,m,uvstar);%u=u_handle(t,x,ustar);
        thetadot=xtheta(9:16);
        thetadotdot=Mthetam\((1/m.ro)*E*m.a*uv-Cthetam*thetadot-G);
        disp('function evaluated at time:');
        disp(t);

        xthetadot=[thetadot; thetadotdot];
 

end









