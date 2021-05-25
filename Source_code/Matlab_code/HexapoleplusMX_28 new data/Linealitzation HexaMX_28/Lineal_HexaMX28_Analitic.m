%% Lineal_HexaMX28_Analitic


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% This function builds A and B matrix to linearize the model, using the
% analitic way instead of finite differences


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% xstar > state vector in vertical equilibrium (ponit of linearitzation)

% uvstar > actuation vector (motor voltages) in vertical equilibrium 
%(point of linearlitzation)

% m > structural parameter containing all geometric variables that are used to 
% define the model (such as gravity constant g, or anchor points Ai, Bi)  


%%%%%%%%%%%%%%%%%%%%%% Outputs %%%%%%%%%%%%%%%%%%%%%%

% A > matrix that used to linearize the system (multiplies error of state)
% A(x-xstar)

% B > matrix that used to linearize the system (multiplies error of actuation)
% B(u-uvstar)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [ A,B ] = Lineal_HexaMX28_Analitic( xstar, uvstar, m )

% Jacobian JEazuvG of maple will be used in this linearitzation

% Main matrices of our dynamic model needed for the linealitzation

H=Hmatrix(xstar);
Ji=Jimatrix(xstar);

E=Ematrix(xstar);
COR=CORmatrix(xstar);
Jidot=Jidotmatrix(xstar,m);

invJi=pinv(Ji);

% Now we build needed matrices for HexaMX_28 model
Hm=[m.c*(1/m.ro)*E,zeros(8,2)];
Cm=[m.b*(1/m.ro)*E,zeros(8,2)];

Mthetam=(H*invJi-Hm);
Cthetam=((COR-H*invJi*Jidot)*invJi-Cm);


JEazuvG=JEazuvGmatrix(xstar,uvstar);

A21=inv(Mthetam)*JEazuvG*invJi;
A22=-(Mthetam\Cthetam);

% Compute A matix (exacte way)
A=[zeros(8),eye(8);A21,A22];

% Compute B matix (exacte way)
B=[zeros(8,6);(Mthetam)\((1/m.ro)*E*m.a)];
        
end