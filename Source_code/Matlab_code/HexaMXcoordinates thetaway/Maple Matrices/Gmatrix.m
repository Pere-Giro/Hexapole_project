function [ G ] = Gmatrix( x )
%GMAT Summary of this function goes here
%   Detailed explanation goes here
G = [0; 0; 0.9933966000e1; 0; 0; 0; sin(x(7) + x(8)) * (-0.196555548672000002e1) + sin(x(7) - x(8)) * (-0.196555548672000002e1); sin(x(7) + x(8)) * (-0.196555548672000002e1) + sin(x(7) - x(8)) * 0.196555548672000002e1;];


end

