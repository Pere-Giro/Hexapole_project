function [ ANAJ ] = FKJacobianAnalitic( p,rpy,m )
%JACOBIANANALITIC Summary of this function goes here
% Uses the "screw jacobian method" to find Jacobian matrix analytically
% implemented with maple

%we make sure that the input vectors will fit the matricial equations
if isrow(p)
    p=p';
end
if isrow(rpy)
    rpy=rpy';
end

R = [cos(rpy(2)) * cos(rpy(3)) -cos(rpy(2)) * sin(rpy(3)) sin(rpy(2)); sin(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) + cos(rpy(1)) * sin(rpy(3)) -sin(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) + cos(rpy(1)) * cos(rpy(3)) -sin(rpy(1)) * cos(rpy(2)); -cos(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) + sin(rpy(1)) * sin(rpy(3)) cos(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) + sin(rpy(1)) * cos(rpy(3)) cos(rpy(1)) * cos(rpy(2));];

Am = [1 0 sin(rpy(2)); 0 cos(rpy(1)) -sin(rpy(1)) * cos(rpy(2)); 0 sin(rpy(1)) cos(rpy(1)) * cos(rpy(2));];

for i=1:6
    r(:,i)=R*m.B(:,i);
    d(:,i)=m.A(:,i)-p-r(:,i);
    Jd(1:3,i)=d(:,i);
    Jd(4:6,i)=cross(r(:,i),(m.A(:,i)-p));
end
Ae=[eye(3),zeros(3);zeros(3),Am];
    


Ft=2*Jd'*Ae;

ANAJ=-Ft;

end
