function [ ANAJ ] = FKJacobianAnaliticSimulink( p,rpy,Ageom,Bgeom )
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

r=zeros(3,6);
d=zeros(3,6);
Jd=zeros(6,6);

for i=1:6
    r(:,i)=R*Bgeom(:,i);
    d(:,i)=Ageom(:,i)-p-r(:,i);
    Jd(1:3,i)=d(:,i);
    Jd(4:6,i)=cross(r(:,i),(Ageom(:,i)-p));
end
Ae=[eye(3),zeros(3);zeros(3),Am];
    


Ft=2*Jd'*Ae;

ANAJ=-Ft;

end
