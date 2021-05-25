function [ u ] = u_function_HexaMX28( t,xtheta,m,uvstar, xthetastar, K )
%U_FUNCTION_LAGRANGEV2 Summary of this function goes here
%   Detailed explanation goes here


global xq
global xbarrax



switch nargin
    case 4
        
%         our home state
%         p0=xq(1:3);
%         rpy0=xq(4:6);
%         [ pf,rpyf ] = FKNewtonMeth( xtheta,p0,rpy0,m );
%         x(1:3)=pf;
%         x(4:6)=rpyf;
%         x(7:8)=xtheta(7:8);
%         wrenchstar=[0,0,(m.mp+m.mb)*m.g,0,0,0]';
%         
% %         calculating Jscrew home position
%         Jscrewstar=Calc_Jscrew_HexaMX28(x);
%         
% %         defining ustar vector
%         utstar=inv(Jscrewstar)*wrenchstar;
%         utaustar=m.ro*utstar;
%         uvstar=(m.Res/(m.Kt*m.EN))*utaustar;
        
        u=uvstar;
        
        
        
    case 6 
        
        anglesstar=xthetastar(1:8,1);
        
        angles=xtheta(1:8,1);
        
        anglesdif=angle_diff(angles,anglesstar);
        
        xbarra=xtheta-xthetastar;
        xbarra(7:8,1)=anglesdif(7:8,1);
        
        xbarrax=horzcat(xbarrax,xbarra);
        
        u=uvstar-K*xbarra;
        

            
    otherwise
        disp('error number of input variables is not correct');
end

end

