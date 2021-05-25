function [ ls ] = theta_to_L_Simulink( xtheta,lc,ro )
%L_TO_THETA Summary of this function goes here
%   Detailed explanation goes here



ls=ro*[xtheta(1),xtheta(2),xtheta(3),xtheta(4),xtheta(5),xtheta(6)]'+lc;


end

