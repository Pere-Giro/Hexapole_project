function [ pf,rpyf] = FKNewtonSimulink( xtheta,p0,rpy0,lc,ro,Ageom,Bgeom)
% NEWTONMETH
% Given a determinate ROW vector of xtheta state vector of our system and a starting 
% position of the platform vectors p0,rpy0, uses Newton method to find the 
% correct position of the platform that matches ls lenghts. FiDif method


%precision you want to have in your solution (consider that By default,
%MATLAB® uses 16 digits of precision. )

ep=1e-12;

ls=theta_to_L_Simulink(xtheta,lc,ro);

%limit number of iterations to avoid infinite loop

iter0=0;
iterlim=15;

%defining our xold coordenates, our starting point, aproximate to the final
%solution
xo=[p0;rpy0];
    
converged=false;

%iteration loop with 1e-9 tolerance error for instance
while not(converged) && iter0<iterlim

    iter0=iter0+1;

    %Fxo will be the value of our function for xold
    Fxo=FKkinematiceq1(ls,xo(1:3,1),xo(4:6,1),Ageom);

    %Fx will be the Jacobian in xold point
    Fx=FKJacobianAnaliticSimulink(xo(1:3,1),xo(4:6,1),Ageom,Bgeom);

    %inverting Jacobian
    iFx=pinv(Fx);

    %solving the linear system
    dx=-iFx*Fxo;

    %updating the value of xold
    xo=xo+dx;

    converged=max(abs(dx))>ep;

end

pf=xo(1:3,1);
rpyf=xo(4:6,1);

    
disp(['number of iter. = '])
disp(iter0)

%DISPLAYING THE FINAL VALUE OF Fxo 
% disp(Fxo)

end

