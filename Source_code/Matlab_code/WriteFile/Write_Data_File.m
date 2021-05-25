%% Write_Data_File


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% The aim of this script is to create the Geom_Data.txt file,
% containig the parameters used to build the model. This text file will be
% read by MATLAB and MAPLE, and will make it easier to import/export the
% symbolic matrices


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% Parameters of the model

ls=0.96724;  % length of the stick that links the ball with the platform,[m]  
mp=0.59895;     % mass of the platform [kg] 
mb=0.41472;   % mass of the ball+stick [kg] 

% Values of the platform tensor

ITpxx=270725.71*1e-9; %XX axis inercia of the platform [kg*m^2] 
ITpyy=270725.71*1e-9; %XX axis inercia of the platform [kg*m^2] 
ITpzz=409645.04*1e-9; %XX axis inercia of the platform [kg*m^2] 

% Values of the pendulum tensor

ITbxx=11262205.59*1e-9; %XX axis inercia of the platform [kg*m^2] 
ITbyy=11262205.59*1e-9; %XX axis inercia of the platform [kg*m^2] 
ITbzz=77397.82*1e-9; %XX axis inercia of the platform [kg*m^2] 

%%%%%%%%%%%%%%%%%%%%%% Outputs %%%%%%%%%%%%%%%%%%%%%% 

% Geom_Data.txt

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Geom_Vector=[ls,mp,mb,ITpxx,ITpyy,ITpzz,ITbxx,ITbyy,ITbzz]';

Geom_Param=to_file('Geom_Data',Geom_Vector); 


%% Reading from file and putting it into geometric parameters

Geom_Vec=from_file('Geom_Data.txt');

m.ls=Geom_Vec(1,1);
m.mp=Geom_Vec(2,1);
m.mb=Geom_Vec(3,1);
m.ITpxx=Geom_Vec(4,1);
m.ITpyy=Geom_Vec(5,1);
m.ITpzz=Geom_Vec(6,1);
m.ITbxx=Geom_Vec(7,1);
m.ITbyy=Geom_Vec(8,1);
m.ITbzz=Geom_Vec(9,1);


