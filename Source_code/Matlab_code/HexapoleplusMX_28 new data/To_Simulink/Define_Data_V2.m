Flag00='YES'; %Flag to set if you want to simulate with %%CONTROL LAW%% or not
%               (Flag == YES)> simulation with %%CONTROL LAW%%
%               (Flag == NO)> simulation wthout %%CONTROL LAW%%

Flag01='PACK';
%               (Flag01=='PACK')> set theta plots in packets
%               (Flag01=='FULL')> set theta plots individualy

Flag1='YES'; %Secondary flag to set if you want to monitorize or not x state (q domain)
%                 (Flag2 == YES)> monitorize x
%                 (Flag2 == NO)> omit monitorizing of x

Flag2='YES'; %Secondary flag to set if you want to monitorize or not the actuation voltages
%                 (Flag2 == YES)> monitorize voltages
%                 (Flag2 == NO)> omit monitorizing voltages

global Flag3
Flag3='YES'; %Secondary flag to set if you want to monitorize or not the actuation cable tensions and motor accelerations
%                 (Flag2 == YES)> monitorize cable tensions and accelerations
%                 (Flag2 == NO)> omit monitorizing cable tensions and accelerations

if strcmp(Flag00, 'YES')
        Flag4='NO';%Secondary flag to set if you want external PERTRUB. or not
%                 (Flag4 == YES)>  external pertrub. ENABLED
%                 (Flag4 == NO)> omit external pertrub.
end        




%reading data from file

        tf = 4;    % final simulation time
        h = 0.001; %number of samples within final vectors (times and states)
        
        m.g=9.8;    % gravity constant [m/s^2] **(used to plot and draw only//main calculus in MAPLE)**
        
        Geom_Vec=from_file('Geom_Data.txt');

        m.ls=Geom_Vec(1,1); % length of the stick that links the ball with the platform,[m]  **(used to plot and draw only//main calculus in MAPLE)**
        m.mp=Geom_Vec(2,1);  % mass of the platform [kg] **(used to plot and draw only//main calculus in MAPLE)**
        m.mb=Geom_Vec(3,1); % mass of the ball+stick [kg] **(used to plot and draw only//main calculus in MAPLE)**
        
        m.ITpxx=Geom_Vec(4,1); %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        m.ITpyy=Geom_Vec(5,1); %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        m.ITpzz=Geom_Vec(6,1); %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
       
        m.ITbxx=Geom_Vec(7,1); %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        m.ITbyy=Geom_Vec(8,1); %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        m.ITbzz=Geom_Vec(9,1); %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        
        %A and B matrices of the hexapole geometry, all distances in [m]
        m.B=1e-3*[0,-89.15,0;0,-89.15,0;77.21,44.57,0;77.21,44.57,0;-77.21,44.57,0;-77.21,44.57,0]';
        m.A=1e-3*[-231.62,-136.18,0;231.62,-136.18,0;233.74,-132.5,0;2.13,268.67,0;-2.13,268.67,0;-233.74,-132.5,0]';
        
        %Defining motor related constants
        m.ro=0.025/2; %radius of the pulley fixed in MX-28 load axis [m]
        
        m.Vs=12; %Source Voltage [V]


        m.Res=8.3; %Resistance [om]
        m.L=2.03e-3; %Inductance [H]
        m.eff=0.836; %Gear efficiency


        m.N=193; %gearbox ratio
        m.Kom=93.0959; %motor speed constant [rad/V] (paper==93.1[rad/V])//(Maxononlineshop== 889 [rpm/V]==>93.0959 [rad/V])
        m.Kt=0.0107; %Torque constant [N*m/A] (paper==1/93.1[N*m/A])//(Maxononlineshop== 10.7 [mN*m/A])
        m.Jm=8.68e-8; %Inertia value at motor axis [kg*m^2]
        m.Jl=2.896e-7; %Inercia volue at load axis [kg*m^2]
        m.bm=8.87e-8; %Friction value at motor axis [N*m*s]
        m.EN=m.N*m.eff; %Constant that relates tauload with taumotor
        
        %equation of MX-28 constants
        m.a=(m.EN*m.Kt)/m.Res; %constant to be used to indroduce MX-28 to DM
        m.b=-(m.EN*(m.Kt)^2*m.N)/m.Res-m.EN*m.bm*m.N; %constant to be used to indroduce MX-28 to DM
        m.c=-m.EN*m.N*m.Jm; %constant to be used to indroduce MX-28 to DM
        
        %cable lenghts
%         m.lc=-1*[0.381915756810321;0.381915756810321;0.381910232646364;0.381913362426611;0.381913362426611;0.381910232646364];%lengths of the cables in home position
%      
 %defining xstar vector 
        xstar(1,1)=0;
        xstar(2,1)=0;
        xstar(3,1)=-0.300;
        xstar(4,1)=0;
        xstar(5,1)=0;
        xstar(6,1)=0;
        xstar(7,1)=0;
        xstar(8,1)=0;
        
        xstar(9,1)=0;
        xstar(10,1)=0;
        xstar(11,1)=0;
        xstar(12,1)=0;
        xstar(13,1)=0;
        xstar(14,1)=0;
        xstar(15,1)=0;
        xstar(16,1)=0;
        
        %cable lenghts
         m.lc=IKsolve(xstar,m);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         % Calculating our utstar (the tensions of the cables that are in
         % our home state)
        wrenchstar=[0,0,(m.mp+m.mb)*m.g,0,0,0]';
        
        %calculating Jscrew home position
        Jscrewstar=Calc_Jscrew_HexaMX28(xstar);
        %defining utaustar vector
        utstar=inv(Jscrewstar)*wrenchstar;
        utaustar=(m.ro)*utstar;
        
        %now we build voltages in our home position vector
        uvstar=(m.Res/(m.Kt*m.EN))*utaustar;
        disp(uvstar)
        


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Bryson Rule
        
        CTE1=1; %constant to adjust position terms of Q matrix
        CTE2=1; %constant to adjust velocity terms of Q matrix
        CTE3=1100; %constant to adjust actuation terms of R matrix
        
        Q1= [ 10/(deg2rad(60))^2*eye(6),  zeros(6,2)                ; ...
                 zeros(2,6)           ,  1/(deg2rad(5))^2 * eye(2) ];
             
             
        Q2= [ (1/50)*((1/((5.6)^2))*eye(6)) ,   zeros(6,2)                ; ...
                 zeros(2,6)                      ,   (1/10)*(1/(deg2rad(5))^2 * eye(2)) ];
             
             
        Q = [ CTE1*Q1,          zeros(8,8)   ; ...
              zeros(8,8),  CTE2*Q2      ];
        
        R=CTE3*(1/(10)^2)*eye(6);
        
        xthetastar(1:8,1)=[0,0,0,0,0,0,xstar(7),xstar(8)]';
        xthetastar(9:16,1)=zeros(8,1);

        [Alin,Blin]=Lineal_HexaMX28_Analitic( xstar, uvstar, m);
        
        [K,SolRic,Eigvals]=lqr(Alin,Blin,Q,R);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%
     
R1 = @(x) [1 0 0; 0 cos(x(4)) -sin(x(4)); 0 sin(x(4)) cos(x(4));]; % Rot_x(psi)
R2 = @(x) [cos(x(5)) 0 sin(x(5)); 0 1 0; -sin(x(5)) 0 cos(x(5));]; % Rot_y(theta)
R3 = @(x) [cos(x(6)) -sin(x(6)) 0; sin(x(6)) cos(x(6)) 0; 0 0 1;]; % Rot_z(phi)
R4 = @(x) [1 0 0; 0 cos(x(7)) -sin(x(7)); 0 sin(x(7)) cos(x(7));]; % Rot_x(beta1)
R5 = @(x) [cos(x(8)) 0 sin(x(8)); 0 1 0; -sin(x(8)) 0 cos(x(8));]; % Rot_t(beta2)




 %initial conditions
         x0=[0,0,-0.3,0,0,0,deg2rad(1),deg2rad(1),0,0,0,0,0,0,0,0]';
        
         global xq
         xq=x0;

         % Time
         t = 0:h:tf;
         
         global xbarrax
         xbarrax(16,1)=0;
         
         % Arrays needed to monitorize cable tensions later
         switch Flag3
             case 'YES'
                 global xdotArray
                 xdotArray(16,1)=0;
                 
                 global tvec
                 tvec(1,1)=0;
         end

         xtheta0=x_to_xtheta(x0,m);
         display(xtheta0);