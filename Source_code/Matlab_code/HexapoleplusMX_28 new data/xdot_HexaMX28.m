%% xdot_HexaMX28


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% This function is the implementation of the differential equations 
% (non-linearized), that modelise our hexapole system (integrating the dynamics 
% of the motor). This function is supposed to be given as an input for MATLAB 
% integration rutines such as ode45 and ode15s.


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% t > evaluation time of the function (the purpose of this input is to know 
% when the function is being evaluated by the integration rutine)

% xtheta > state vector in a particular instant of time, given in motor
% coordinates

% m > structural parameter containing all geometric variables that are used to 
% define the model (such as gravity constant g, or anchor points Ai, Bi)  

% uvstar > actuation vector (motor voltages) in vertical equilibrium 
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

function [ xthetadot ] = xdot_HexaMX28( t, xtheta, m, uvstar, xthetastar, K, uo_handle)

% global xthetaplot
% xthetaplot=horzcat(xthetaplot,xtheta);

global xq

global xdotArray

global Flag3

% vector of times necesary to later monitorize cable tensions
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

% Now we build needed matrices for HexaMX_28 model
Hm=[m.c*(1/m.ro)*E,zeros(8,2)];
Cm=[m.b*(1/m.ro)*E,zeros(8,2)];

Mthetam=(H*invJi-Hm);
Cthetam=((COR-H*invJi*Jidot)*invJi-Cm);


switch nargin
    case 4
        uv=u_function_HexaMX28(t,xtheta,m,uvstar);
        thetadot=xtheta(9:16);
        thetadotdot=Mthetam\((1/m.ro)*E*m.a*uv-Cthetam*thetadot-G);
        disp('function evaluated at time:');
        disp(t);

        xthetadot=[thetadot; thetadotdot];
 
    case 6
        uv=u_function_HexaMX28(t, xtheta, m, uvstar, xthetastar, K); 
        thetadot=xtheta(9:16);
        thetadotdot=Mthetam\((1/m.ro)*E*m.a*uv-Cthetam*thetadot-G);
        disp('function evaluated at time:');
        disp(t);

        xthetadot=[thetadot; thetadotdot];

        switch Flag3
            case 'YES'
                xdotArray=horzcat(xdotArray,xthetadot);
        end
        
    case 7
        
        Eo=Eomatrix(x);
        
        uv=u_function_HexaMX28(t,xtheta,m,uvstar,xthetastar,K);
        uo=uo_handle(t);
        thetadot=xtheta(9:16);
        thetadotdot=Mthetam\((1/m.ro)*E*m.a*uv+Eo*uo-Cthetam*thetadot-G);
        disp('function evaluated at time:');
        disp(t);

        xthetadot=[thetadot; thetadotdot];
        
        switch Flag3
            case 'YES'
                xdotArray=horzcat(xdotArray,xthetadot);
        end
        
        
    otherwise
        disp('error in number of input variables');
end

end