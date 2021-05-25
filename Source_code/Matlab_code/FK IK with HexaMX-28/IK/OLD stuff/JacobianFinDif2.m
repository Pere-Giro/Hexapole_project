function [ DIFJ ] = JacobianFinDif2( ls,p,rpy,A )
%UNTITLED Summary of this function goes here
%Uses the finite differences method to find the Jacobian matrix of the
%eq.system Oi=lsi^2-(norm(pi+R*bi-ai))^2 


epsilon=1e-10;

Fxo=kinematiceq2(ls,p,rpy,A);

%derivative with respect to ls 
Z1=zeros(6,1);
J1(6,6)=0;
for i=1:6
    Z1(i,1)=epsilon;
    K1=kinematiceq2(ls+Z1,p,rpy,A);
    J1(:,i)=(K1-Fxo)./epsilon;
    Z1=zeros(6,1);
end




%derivative with respect to p (not necesary cte)

% J2=[(kinematiceq1(ls,p+[epsilon,0,0]',rpy,A,B)-Fxo),(kinematiceq1(ls,p+[0,epsilon,0]',rpy,A,B)-Fxo),(kinematiceq1(ls,p+[0,0,epsilon]',rpy,A,B)-Fxo)]./epsilon;

%This is the same as above but using a loop
%Z2=zeros(3,1);
% for i=1:3
%     Z2(i,1)=epsilon;
%     K2=kinematiceq1(ls,p+Z2',rpy);
%     J2(:,i)=(K2-Fxo)./epsilon;
%     Z2=zeros(3,1);
% end

%J2



%derivative with respect to rpy (not necesary cte)

% J3=[(kinematiceq1(ls,p,rpy+[epsilon,0,0]',A,B)-Fxo),(kinematiceq1(ls,p,rpy+[0,epsilon,0]',A,B)-Fxo),(kinematiceq1(ls,p,rpy+[0,0,epsilon]',A,B)-Fxo)]./epsilon;

%This is the same as above but using a loop
% Z3=zeros(3,1);
% for i=1:3
%     Z3(i,1)=epsilon;
%     K3=kinematiceq1(ls,p,rpy+Z3');
%     J3(:,i)=(K3-Fxo)./epsilon;
%     Z3=zeros(3,1);
% end

%J3

DIFJ=J1;
    
    
        
        

    

end

