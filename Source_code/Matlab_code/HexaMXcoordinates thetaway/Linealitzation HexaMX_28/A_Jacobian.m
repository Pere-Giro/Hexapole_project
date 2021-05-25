function [ A ] = A_Jacobian( x )
%A_JACOBIAN Summary of this function goes here
%   Detailed explanation goes here

%Main matrices of our dynamic model needed for the linealitzation
H=Hmatrix(x);
Ji=Jimatrix(x);


E=(1/m.ro)*Ematrix(x);
COR=CORmatrix(x);
Jidot=Jidotmatrix(x,m);

invJi=pinv(Ji);

Mtheta=H*invJi;
Ctheta=(COR-H*invJi*Jidot)*invJi;

JEtauG=JEtauGmatrix(xstar,[0.0263448223642549;0.0263448223642549;0.0263477431719215;0.0263472661521605;0.0263472661521605;0.0263477431719215]);
A21=inv(Mtheta)*JEtauG*invJi;
A22=-inv(Mtheta)*Ctheta;

%Compute A matrix analiticali
A=[zeros(8),eye(8);A21,A22];

end

