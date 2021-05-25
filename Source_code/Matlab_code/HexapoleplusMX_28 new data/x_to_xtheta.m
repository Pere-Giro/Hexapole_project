function [ xtheta ] = x_to_xtheta( x,m,Ji )
%X_TO_XTHETA Summary of this function goes here
%   Detailed explanation goes here

switch nargin
    case 2
        Ji=Jimatrix(x);
end

    
lf=IKsolve(x,m);

theta6=L_to_theta(lf,m);

thetadot8=Ji*x(9:16);

xtheta=[theta6;x(7);x(8);thetadot8];





end

