%% Define motor constants and general data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%WARNING A,B,lcbase,robase, will only be used here if you wnt to change
%model's you need to enter xdot_Simulink

%A and B matrices of the hexapole geometry, all distances in [m] 
% Bgeom=1e-3*[0,-89.15,0;0,-89.15,0;77.21,44.57,0;77.21,44.57,0;-77.21,44.57,0;-77.21,44.57,0]';
lcbase=-1*[0.381915756810321;0.381915756810321;0.381910232646364;0.381913362426611;0.381913362426611;0.381910232646364];%lengths of the cables in home position
robase=0.025/2;
A=1e-3*[-231.62,-136.18,0;231.62,-136.18,0;233.74,-132.5,0;2.13,268.67,0;-2.13,268.67,0;-233.74,-132.5,0]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initial conditions
xic=[0;0;-0.3;0;0;0;0;0;0;0;0;0;0;0;0;0];
xthetaIC=x_to_xtheta_Simulink(xic,A,robase,lcbase); %WARNING x_to_xtheta function is required in this line be sure to have this function in matlab path

xq=Simulink.Signal;

xq.DataType='double';

xq.Complexity='real';

xq.Dimensions=[16 1];

xq.InitialValue='xthetaIC';
%Home state

xstar=[0;0;-0.3;0;0;0;0;0;0;0;0;0;0;0;0;0];
xthetastar=x_to_xtheta_Simulink(xstar,A,robase,lcbase);

%Home actuation 'Definig ustar with all maple data (mass and inertia
%predefined)'
uvstar=[0.126656068431476;0.126656068431476;0.126670110584070;0.126667817251185;0.126667817251185;0.126670110584070];

%K matrix of the control law (current K is form 06-06-18 simulation// see
%my folder videos d'animacions)

K=[0.139984643683273 0.308075654045057 0.152593980972437 -0.152658574980171 0.152661454755908 -0.150259883513785 -0.00758923405034467 739.080455703725 -1.43251333259100 1.58328870378885 0.791636306727144 -0.791615033127853 0.791651363296824 -0.791633155889286 -0.00241220176486092 237.323400456421;0.308075654044011 0.139984643683042 -0.150259883516444 0.152661454755837 -0.152658574980579 0.152593980970565 -0.00758923911023507 -739.080455703625 1.58328870378858 -1.43251333259078 -0.791633155898373 0.791651363305865 -0.791615033118571 0.791636306717853 -0.00241220332572177 -237.323400456385;0.152660186889717 -0.150259882566551 0.139985539764601 0.308077238402338 0.152593637605212 -0.152655726034739 -640.058657046317 -369.547294006834 0.791644875161497 -0.791633155765954 -1.43250895034913 1.58329730369768 0.791635243674520 -0.791600988876380 -205.526881054011 -118.663947781055;-0.152658573478606 0.152595245838448 0.308077237302787 0.139980521146593 -0.150260914164539 0.152659844559035 640.071495017025 369.537177636335 -0.791615032762749 0.791642794274470 1.58329730393286 -1.43253491028256 -0.791638584654670 0.791643812804876 205.530978667618 118.660742218559;0.152595245835631 -0.152658573477138 0.152659844558818 -0.150260914164523 0.139980521146931 0.308077237301077 640.071495014284 -369.537177631764 0.791642794264948 -0.791615032753246 0.791643812804614 -0.791638584654493 -1.43253491027285 1.58329730392318 205.530978666780 -118.660742217155;-0.150259882563224 0.152660186888101 -0.152655726034204 0.152593637604793 0.308077238402416 0.139985539765293 -640.058657043668 369.547294002537 -0.791633155756983 0.791644875152544 -0.791600988876002 0.791635243674227 1.58329730368838 -1.43250895033995 -205.526881053186 118.663947779737];

%%%%%%%%%%%%%%%%%%%%%%%%%%
Geom_Vec=from_file('Geom_Data.txt');

%to get x_Simulink.mat into Array do get(x_states_Array,'Data')

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
        m.lc=-1*[0.381915756810321;0.381915756810321;0.381910232646364;0.381913362426611;0.381913362426611;0.381910232646364];%lengths of the cables in home position
     
 m.g=9.8; 
        

