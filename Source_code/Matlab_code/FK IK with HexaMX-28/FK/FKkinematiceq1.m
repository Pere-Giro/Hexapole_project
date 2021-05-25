function [ O ] = FKkinematiceq1( ls,p,rpy,A )
%KINEMATICEQ1 Summary of this function goes here
% A simpler version of kinematiceq with one output O, easier to use
% with other functions 
%ls == length of the cables
%p == position vector of the central point of the platrom in ABS ref.
%rpy == euler angles to describe the orientation of the platform
%O is the value of the equation (F(x)=0; O=ls^2-(norm(ai-p-R*bi))^2) [MAIN OBJECTIVE
%OF THIS FUNCTION]


%we make sure that the input vectors will fit the matricial equations
% if isrow(ls)
%     ls=ls';
% end
% if isrow(p)
%     p=p';
% end
% if isrow(rpy)
%     rpy=rpy';
% end

%biabs == matrix with all the positions vectors that were B matrix, but now
%in ABS ref. (matrix of platform position in ABS ref.)
biabs = [p(1) + cos(rpy(2)) * sin(rpy(3)) * 0.891500000000000070e-1 p(1) + cos(rpy(2)) * sin(rpy(3)) * 0.891500000000000070e-1 p(1) + cos(rpy(2)) * cos(rpy(3)) * 0.772100000000000009e-1 + cos(rpy(2)) * sin(rpy(3)) * (-0.445699999999999985e-1) p(1) + cos(rpy(2)) * cos(rpy(3)) * 0.772100000000000009e-1 + cos(rpy(2)) * sin(rpy(3)) * (-0.445699999999999985e-1) p(1) + cos(rpy(2)) * cos(rpy(3)) * (-0.772100000000000009e-1) + cos(rpy(2)) * sin(rpy(3)) * (-0.445699999999999985e-1) p(1) + cos(rpy(2)) * cos(rpy(3)) * (-0.772100000000000009e-1) + cos(rpy(2)) * sin(rpy(3)) * (-0.445699999999999985e-1); p(2) + sin(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * 0.891500000000000070e-1 + cos(rpy(1)) * cos(rpy(3)) * (-0.891500000000000070e-1) p(2) + sin(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * 0.891500000000000070e-1 + cos(rpy(1)) * cos(rpy(3)) * (-0.891500000000000070e-1) p(2) + sin(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) * 0.772100000000000009e-1 + cos(rpy(1)) * sin(rpy(3)) * 0.772100000000000009e-1 + sin(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * (-0.445699999999999985e-1) + cos(rpy(1)) * cos(rpy(3)) * 0.445699999999999985e-1 p(2) + sin(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) * 0.772100000000000009e-1 + cos(rpy(1)) * sin(rpy(3)) * 0.772100000000000009e-1 + sin(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * (-0.445699999999999985e-1) + cos(rpy(1)) * cos(rpy(3)) * 0.445699999999999985e-1 p(2) + sin(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) * (-0.772100000000000009e-1) + cos(rpy(1)) * sin(rpy(3)) * (-0.772100000000000009e-1) + sin(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * (-0.445699999999999985e-1) + cos(rpy(1)) * cos(rpy(3)) * 0.445699999999999985e-1 p(2) + sin(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) * (-0.772100000000000009e-1) + cos(rpy(1)) * sin(rpy(3)) * (-0.772100000000000009e-1) + sin(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * (-0.445699999999999985e-1) + cos(rpy(1)) * cos(rpy(3)) * 0.445699999999999985e-1; p(3) + cos(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * (-0.891500000000000070e-1) + sin(rpy(1)) * cos(rpy(3)) * (-0.891500000000000070e-1) p(3) + cos(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * (-0.891500000000000070e-1) + sin(rpy(1)) * cos(rpy(3)) * (-0.891500000000000070e-1) p(3) + cos(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) * (-0.772100000000000009e-1) + sin(rpy(1)) * sin(rpy(3)) * 0.772100000000000009e-1 + cos(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * 0.445699999999999985e-1 + sin(rpy(1)) * cos(rpy(3)) * 0.445699999999999985e-1 p(3) + cos(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) * (-0.772100000000000009e-1) + sin(rpy(1)) * sin(rpy(3)) * 0.772100000000000009e-1 + cos(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * 0.445699999999999985e-1 + sin(rpy(1)) * cos(rpy(3)) * 0.445699999999999985e-1 p(3) + cos(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) * 0.772100000000000009e-1 + sin(rpy(1)) * sin(rpy(3)) * (-0.772100000000000009e-1) + cos(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * 0.445699999999999985e-1 + sin(rpy(1)) * cos(rpy(3)) * 0.445699999999999985e-1 p(3) + cos(rpy(1)) * sin(rpy(2)) * cos(rpy(3)) * 0.772100000000000009e-1 + sin(rpy(1)) * sin(rpy(3)) * (-0.772100000000000009e-1) + cos(rpy(1)) * sin(rpy(2)) * sin(rpy(3)) * 0.445699999999999985e-1 + sin(rpy(1)) * cos(rpy(3)) * 0.445699999999999985e-1;];

%XX is the second part of the equation we want to evaluate F(x)=0, it is a column matrix, the
%first part is: (lengths of the cables)^2 == ls^2 
XX=zeros(6,1);
for i=1:6
    XX(i,1)=(norm(A(:,i)-biabs(:,i))).^2;
end

%value of the function F(x)
O=XX-ls.^2;

end

