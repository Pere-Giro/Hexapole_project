%% uo_pert_wrenc_MXcord


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% The aim of this function is to introduce non-modalized perturbations of force
% in a particular instant of time


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% t > evaluation time of the function (the purpose of this input is to know 
% in which instant of time the perturbation has to be introduced)


%%%%%%%%%%%%%%%%%%%%%% Outputs %%%%%%%%%%%%%%%%%%%%%%

% uo > vector of perturbation force

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ uo ] = uo_pert_wrench_MXcord( t )
%UO_PERT_WRENCH Summary of this function goes here
%   Detailed explanation goes here

if  t<5.8 
    uo=[0,0,0]';
    
elseif (t>=5.8 && t<5.9)
    uo=[0.5, 0, 0]';
    
elseif (t>=9.5 && t<9.6)
    uo=[0, 0.5, 0]';
    
elseif (t>=13 && t<13.1)
    uo=[0, 0, -2]';
    
else
    uo=[0,0,0]';
    
end

end