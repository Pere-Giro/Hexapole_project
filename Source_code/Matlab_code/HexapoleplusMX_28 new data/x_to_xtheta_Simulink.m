function [ xtheta ] = x_to_xtheta_Simulink( x,Ageom,ro,lc,Ji )
%X_TO_XTHETA Summary of this function goes here
%   Detailed explanation goes here

switch nargin
    case 4
        Ji=Jimatrix(x);
end

    
lf=IKsolve_Simulink(x,Ageom);

theta6=L_to_theta_Simulink(lf,ro,lc);

thetadot8=Ji*x(9:16);

xtheta=[theta6;x(7);x(8);thetadot8];





end

