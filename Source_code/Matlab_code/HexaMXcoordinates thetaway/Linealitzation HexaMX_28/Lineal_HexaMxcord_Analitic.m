%% Lineal_HexaMxcord_Analitic


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% This function builds A and B matrix to linearize the model, using the
% analitic way instead of finite differences


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% xstar > state vector in vertical equilibrium (ponit of linearitzation)

% utaustar > actuation vector (motor torques) in vertical equilibrium 
%(point of linearlitzation)

% m > structural parameter containing all geometric variables that are used to 
% define the model (such as gravity constant g, or anchor points Ai, Bi)  


%%%%%%%%%%%%%%%%%%%%%% Outputs %%%%%%%%%%%%%%%%%%%%%%

% A > matrix that used to linearize the system (multiplies error of state)
% A(x-xstar)

% B > matrix that used to linearize the system (multiplies error of actuation)
% B(u-ustar)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ A,B ] = Lineal_HexaMxcord_Analitic( xstar, utaustar, m )

% Main matrices of our dynamic model needed for the linealitzation
H=Hmatrix(xstar);
Ji=Jimatrix(xstar);


E=(1/m.ro)*Ematrix(xstar);
COR=CORmatrix(xstar);
Jidot=Jidotmatrix(xstar,m);

invJi=pinv(Ji);

Mtheta=H*invJi;
Ctheta=(COR-H*invJi*Jidot)*invJi;

JEtauG=JEtauGmatrix(xstar,utaustar);
A21=inv(Mtheta)*JEtauG*invJi;
A22=-inv(Mtheta)*Ctheta;


% Compute A matrix analiticali
A=[zeros(8),eye(8);A21,A22];

% Compute B matix (exacte way)
B=[zeros(8,6);(Mtheta)\E];
        
end

