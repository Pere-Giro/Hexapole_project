function [ xq ] = xtheta_to_xq( xtheta,xq0,m )
%XTHETA_TO_X Summary of this function goes here
%   Detailed explanation goes here

p0=xq0(1:3);
rpy0=xq0(4:6);




[ pf,rpyf ] = FKfsolve( xtheta,p0,rpy0,m );

xq(1:8,1)=[pf;rpyf;xtheta(7:8)];

Ji=Jimatrix(xq);



xq(9:16,1)=Ji\xtheta(9:16);




end

