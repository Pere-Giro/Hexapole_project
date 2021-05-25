

function d = angle_diff(alpha,beta)
% This function returns the angular difference alpha-beta (1st argument 
% minus second argument), wrapped to the [-pi,pi] interval. If alpha and 
% beta are vectors, it returns a vector with the element-wise angular 
% differences alpha(i)-beta(i).
%
% Note that Matlab implements angdiff, which returns beta-alpha instead.
% The Robotics Toolbox implements angdiff too, but returns alpha-beta,
% as we do.

    incr = alpha-beta;
    d = atan2(sin(incr),cos(incr));

end