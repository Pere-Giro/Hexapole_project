function [ uo ] = uo_pert_wrench_HexaMX28( t )
%UO_PERT_WRENCH Summary of this function goes here
%   Detailed explanation goes here

% if  t<5 
%     uo=[0,0,0]';
%     
% elseif (t>=5 && t<5.05) %making the perturbation as a slop instead of a step
% %     uo=[0.5,0,0]';
%     uo=[(0.5/(50e-3))*(t-5),0,0]';
%     
% elseif (t>=20 && t<20.05)
%     uo=[0,0,2]';
% 
% else
%     uo=[0,0,0]';
%     
% end

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

