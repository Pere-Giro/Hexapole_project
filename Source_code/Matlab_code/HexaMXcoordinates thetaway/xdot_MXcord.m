%% xdot_MXcord


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% This function is the implementation of the differential equations 
% (non-linearized), that modelise our hexapole system. This function is
% supposed to be given as an input for MATLAB integration rutines such as
% ode45 and ode15s.


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% t > evaluation time of the function (the purpose of this input is to know 
% when the function is being evaluated by the integration rutine)

% xtheta > state vector in a particular instant of time, given in motor
% coordinates

% m > structural parameter containing all geometric variables that are used to 
% define the model (such as gravity constant g, or anchor points Ai, Bi)  

% utaustar > actuation vector (motor torques) in vertical equilibrium 
%(point of linearlitzation)

% xthetastar > state vector in vertical equilibrium (point of
% linearlitzation) given in motor coordinates

% K > matrix given by LQR solution, this matrix is used to build the
% control law [u = ustar -K(x-xstar)]

% uo_handle > handle to the funtion that introduces non-modalized
% perturbations in our model in a particular instant of time


%%%%%%%%%%%%%%%%%%%%%% Outputs %%%%%%%%%%%%%%%%%%%%%%

% xthetadot > time derivative of the state vector, given in motor
% coordinates

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ xthetadot ] = xdot_MXcord( t, xtheta, m, utaustar, xthetastar, K, uo_handle)

global Flag3;

global xq

p0=xq(1:3);
rpy0=xq(4:6);

[ pf,rpyf ] = FKfsolve( xtheta,p0,rpy0,m );

x(1:8,1)=[pf;rpyf;xtheta(7:8)];

H=Hmatrix(x);
G=Gmatrix(x);
E=Ematrix(x);
Ji=Jimatrix(x);

x(9:16,1)=Ji\xtheta(9:16);

switch Flag3
    case 'YES' 
        global xthetaplot;
        xthetaplot=horzcat(xthetaplot,xtheta);
        
        global xqplot;
        xqplot=horzcat(xqplot,x);
end

xq=x;

COR=CORmatrix(x);
Jidot=Jidotmatrix(x,m);


invJi=pinv(Ji);

Mtheta=H*invJi;
Ctheta=(COR-H*invJi*Jidot)*invJi;


switch nargin
    case 4
        u=u_function_MXcord(t,xtheta,m,utaustar); 
        thetadot=xtheta(9:16);
        thetadotdot=Mtheta\(E*(1/m.ro)*u-Ctheta*thetadot-G);
        
        disp('function evaluated at time:');
        disp(t);
        xthetadot=[thetadot; thetadotdot];
 
    case 6
        u=u_function_MXcord(t,xtheta,m,utaustar,xthetastar,K);
        thetadot=xtheta(9:16);
        thetadotdot=Mtheta\(E*(1/m.ro)*u-Ctheta*thetadot-G);
        
        disp('function evaluated at time:');
        disp(t);
        
        xthetadot=[thetadot; thetadotdot];
        
    case 7
          
        Eo=Eomatrix(x);
        
        u=u_function_MXcord(t,xtheta,m,utaustar,xthetastar,K);
        uo=uo_handle(t);
        thetadot=xtheta(9:16);
        thetadotdot=Mtheta\((1/m.ro)*E*u+Eo*uo-Ctheta*thetadot-G);
        disp('function evaluated at time:');
        disp(t);

        xthetadot=[thetadot; thetadotdot];
        
        
    otherwise
        disp('error in number of input variables');
end

end