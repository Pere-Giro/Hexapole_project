function [ DIFJ ] = FKJacobianFinDif( ls,p,rpy,A )
%UNTITLED Summary of this function goes here
%Uses the finite differences method to find the Jacobian matrix of the
%eq.system Oi=lsi^2-(norm(pi+R*bi-ai))^2 


epsilon=1e-10;

Fxo=FKkinematiceq1(ls,p,rpy,A);

%derivative with respect to p

% J2=[(kinematiceq1(ls,p+[epsilon,0,0]',rpy,A)-Fxo),(kinematiceq1(ls,p+[0,epsilon,0]',rpy,A)-Fxo),(kinematiceq1(ls,p+[0,0,epsilon]',rpy,A)-Fxo)]./epsilon;

%This is the same as above but using a loop
Z2=zeros(3,1);
for i=1:3
    Z2(i,1)=epsilon;
    K2=FKkinematiceq1(ls,p+Z2,rpy,A);
    J2(:,i)=(K2-Fxo)./epsilon;
    Z2=zeros(3,1);
end




%derivative with respect to rpy

% J3=[(kinematiceq1(ls,p,rpy+[epsilon,0,0]',A)-Fxo),(kinematiceq1(ls,p,rpy+[0,epsilon,0]',A)-Fxo),(kinematiceq1(ls,p,rpy+[0,0,epsilon]',A)-Fxo)]./epsilon;

% This is the same as above but using a loop

Z3=zeros(3,1);
for i=1:3
    Z3(i,1)=epsilon;
    K3=FKkinematiceq1(ls,p,rpy+Z3,A);
    J3(:,i)=(K3-Fxo)./epsilon;
    Z3=zeros(3,1);
end



DIFJ=[J2,J3];
    
    
        
        

    

end

