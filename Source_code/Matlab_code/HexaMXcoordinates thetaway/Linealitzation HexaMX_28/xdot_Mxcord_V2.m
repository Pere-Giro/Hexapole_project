function [ xthetadot ] =xdot_Mxcord_V2( t, xtheta, m, ustar)
%XDOT_HEXAPOLE Summary of this function goes here
%   differential equation (non-linearized), that modelises our hexapole
%   system


p0=[0,0,-0.3]';
rpy0=[0,0,0]';
%[pf,rpyf] = FKNewtonMeth( xtheta,p0,rpy0,m,'Analytic' );
[ pf,rpyf ] = FKfsolve( xtheta,p0,rpy0,m );
x(1:3)=pf;
x(4:6)=rpyf;
x(7:8)=xtheta(7:8);

H=Hmatrix(x);
G=Gmatrix(x);
E=Ematrix(x);
Ji=Jimatrix(x);


x(9:16)=inv(Ji)*xtheta(9:16);


COR=CORmatrix(x);
Jidot=Jidotmatrix(x,m);


% Cmx=[E*m.b*(1/m.ro),zeros(8,2)];
% Mmx=[E*m.c*(1/m.ro),zeros(8,2)];
% Mtheta=H*inv(Ji)-Mmx;
% Ctheta=(COR-H*inv(Ji)*Jidot)*inv(Ji)-Cmx;

Mtheta=H*pinv(Ji);
Ctheta=(COR-H*pinv(Ji)*Jidot)*pinv(Ji);

        u=ustar;
        thetadot=xtheta(9:16);
        thetadotdot=Mtheta\(E*(1/m.ro)*u-Ctheta*thetadot-G);
        disp('function evaluated at time:');
        disp(t);

        xthetadot=[thetadot; thetadotdot];
        



end









