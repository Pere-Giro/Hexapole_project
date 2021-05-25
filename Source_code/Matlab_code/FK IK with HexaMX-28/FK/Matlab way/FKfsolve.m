function [ pf,rpyf ] = FKfsolve( xtheta,p0,rpy0,m )
% NEWTONMETH
% Given a determinate ROW vector of xtheta state vector of our system and a starting 
% position of the platform vectors p0,rpy0, uses Newton method to find the 
% correct position of the platform that matches ls lenghts. FiDif method
%activateing profile to track execution time
%profile on

ls=theta_to_L(xtheta,m);
%Fxo will be the value of our function for xold
fun=@(x)FKkinematiceq1(ls,x(1:3),x(4:6),m.A);
xo=[p0;rpy0];
% options = options('fsolve','Display','off',' FunctionTolerance',1e-20);
% optimoptions('fsolve','Display','off',' TolFun',1e-20)
options = optimset('Display','off','TolFun',1e-6);
x = fsolve(fun,xo,options);


%WARNING this patch may cause future problems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for i=1:6
%     if abs(x(i,1))<1e-7
%         x(i,1)=0;
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pf=x(1:3);
rpyf=x(4:6);





end

