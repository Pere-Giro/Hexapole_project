function [ xbarra ] = xbarra_Simulink( xthetastar,xtheta )
%XBARRA_SIMULINK Summary of this function goes here
%   Detailed explanation goes here

        anglesstar=xthetastar(1:8,1);
        
        angles=xtheta(1:8,1);
        
        anglesdif=angle_diff(angles,anglesstar);
        
        xbarra1=xtheta-xthetastar;
        xbarra=[xbarra1(1:6,1);anglesdif(7:8,1);xbarra1(9:16,1)];
        
end

