function [ A,B ] = Lineal_HexaMxcord_FinDif( xtheta, ustar, m, xstar,flag  )
%LINAL_LAG_FINDIF Summary of this function goes here
%   Implementation of Alin (FinDif) and Blin(exacte way), this version will use central finite diferences to copmute Alin 
t=1;

epsilon=1e-6;
% epsilon2=1e-6;

%Warning xdot_Mxcord_V2 has been changed for xdot_MXcord
switch nargin
    case 4
        disp('A centralfindif B exact')
        Z2=zeros(16,1);

        %Central FinDif
        %derivative with respect to xtheta

        Z2=zeros(16,1);
        for i=1:16
            Z2(i,1)=epsilon;
            Fxh=xdot_Mxcord_V2(t, xtheta+Z2, m, ustar);
            Fx_h=xdot_Mxcord_V2(t,xtheta-Z2, m, ustar);
            A(:,i)=(Fxh-Fx_h)./(2*epsilon);
            Z2=zeros(16,1);
        end
        
        H=Hmatrix(xstar);
        Ji=Jimatrix(xstar);
        Mtheta=H*inv(Ji);
        E=(1/m.ro)*Ematrix(xstar);

        B=[zeros(8,6);(Mtheta)\E];
        
    case 5
        
        switch flag
            case 'normalA'
                disp('A normal findif Bexact way')
                %Normal FinDif

                Fxo=xdot_Mxcord_V2(t, xtheta, m, ustar);

                %derivative with respect to theta
                Z2=zeros(16,1);
                for i=1:16
                    Z2(i,1)=epsilon;
                    K2=xdot_Mxcord_V2(t, xtheta+Z2, m, ustar);
                    AlinF(:,i)=(K2-Fxo)./epsilon;
                    Z2=zeros(16,1);
                end
                H=Hmatrix(xstar);
                Ji=Jimatrix(xstar);
                Mtheta=H*inv(Ji);
                E=(1/m.ro)*Ematrix(xstar);

                B=[zeros(8,6);(Mtheta)\E];

                % AlinF
            case 'allFinDif'
                disp('all matrices computed via FinDif central differences')
                
                Z2=zeros(16,1);
                for i=1:16
                    Z2(i,1)=epsilon;
                    Fxh=xdot_Mxcord_V2(t, xtheta+Z2, m, ustar);
                    Fx_h=xdot_Mxcord_V2(t,xtheta-Z2, m, ustar);
                    A(:,i)=(Fxh-Fx_h)./(2*epsilon);
                    Z2=zeros(16,1);
                end
                
                %derivative with respect to u
                Z3=zeros(6,1);
                for i=1:6
                    Z3(i,1)=epsilon;
                    Fuh=xdot_Mxcord_V2(t, xtheta, m, ustar+Z3);
                    Fu_h=xdot_Mxcord_V2(t, xtheta, m, ustar-Z3);
                    B(:,i)=(Fuh-Fx_h)./(2*epsilon);
                    Z3=zeros(6,1);
                end
  
            otherwise
                disp('this flag does not exist')
        end
    otherwise
    disp('error number of inputs')
end




%derivative with respect to u
% % Z3=zeros(6,1);
% % for i=1:6
% %     Z3(i,1)=epsilon2;
% %     K3=xdot_Mxcord_V2(t, x, m, ustar+Z3);
% %     BlinF(:,i)=(K3-Fxo)./epsilon2;
% %     Z3=zeros(6,1);
% % end

% BlinF




end

