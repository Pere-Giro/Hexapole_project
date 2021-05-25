function [ Eo ] = Eomatrix( x )
%EOMATRIX Summary of this function goes here
%   Detailed explanation goes here

Eo = [1 0 0; 0 1 0; 0 0 1; 0 0 0; 0 0 0; 0 0 0; 0 -0.96724e0 * cos(x(7)) * cos(x(8)) -0.96724e0 * sin(x(7)) * cos(x(8)); 0.96724e0 * cos(x(8)) 0.96724e0 * sin(x(7)) * sin(x(8)) -0.96724e0 * cos(x(7)) * sin(x(8));];


end

