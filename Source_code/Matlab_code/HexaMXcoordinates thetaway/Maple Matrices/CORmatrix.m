function [ COR ] = CORmatrix( x )
%CORMATRIX Summary of this function goes here
%   Detailed explanation goes here
COR = [0 0 0 0 0 0 0 -0.4011337728e0 * x(16) * sin(x(8)); 0 0 0 0 0 0 0.4011337728e0 * x(15) * sin(x(7)) * cos(x(8)) + 0.4011337728e0 * x(16) * cos(x(7)) * sin(x(8)) 0.4011337728e0 * x(15) * cos(x(7)) * sin(x(8)) + 0.4011337728e0 * x(16) * sin(x(7)) * cos(x(8)); 0 0 0 0 0 0 -0.4011337728e0 * x(15) * cos(x(7)) * cos(x(8)) + 0.4011337728e0 * x(16) * sin(x(7)) * sin(x(8)) 0.4011337728e0 * x(15) * sin(x(7)) * sin(x(8)) - 0.4011337728e0 * x(16) * cos(x(7)) * cos(x(8)); 0 0 0 0.1389193300e-3 * x(13) * cos(x(5)) * sin(x(5)) 0.1389193300e-3 * x(12) * cos(x(5)) * sin(x(5)) + 0.2048225200e-3 * cos(x(5)) * x(14) 0.2048225200e-3 * x(13) * cos(x(5)) 0 0; 0 0 0 -0.1389193300e-3 * x(12) * cos(x(5)) * sin(x(5)) - 0.2048225200e-3 * cos(x(5)) * x(14) 0 -0.2048225200e-3 * x(12) * cos(x(5)) 0 0; 0 0 0 0.2048225200e-3 * x(13) * cos(x(5)) 0.2048225200e-3 * x(12) * cos(x(5)) 0 0 0; 0 0 0 0 0 0 -0.3991774382e0 * x(16) * cos(x(8)) * sin(x(8)) -0.3991774382e0 * x(15) * cos(x(8)) * sin(x(8)); 0 0 0 0 0 0 0.3991774382e0 * x(15) * cos(x(8)) * sin(x(8)) 0;];


end

