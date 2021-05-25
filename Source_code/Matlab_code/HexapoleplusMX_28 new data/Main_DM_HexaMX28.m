%% Main_DM_HexaMX28


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% This is the main script of the simulation, by executing this file with
% the proper settings you cant run the desired simualtion


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

Flag='NEW_VIDEO'; % Flag to define what you want the script to do 
%               (Flag == ODE45)> integrate with ode45
%               (Flag == VIDEO)> output a video of an animation of the
%                dynamic model (Only works if Flag1 is set to 'YES')
%               (Flag == NEW_VIDEO)> output a video of an animation of the 
%                dynamic model with STL figure (Only works if Flag1 is set to 'YES')
%               (Flag ==FinDif)> output AlinF and BlinF matrices to build 
%                linealized model through FinDif

Flag00='YES'; % Flag to set if you want to simulate with %%CONTROL LAW%% or not
%               (Flag == YES)> simulation with %%CONTROL LAW%%
%               (Flag == NO)> simulation wthout %%CONTROL LAW%%

Flag01='PACK';
%               (Flag01=='PACK')> set theta plots in packets
%               (Flag01=='FULL')> set theta plots individualy

Flag1='YES'; % Secondary flag to set if you want to monitorize or 
%              not x state (q domain)
%                 (Flag2 == YES)> monitorize x
%                 (Flag2 == NO)> omit monitorizing of x

Flag2='YES'; % Secondary flag to set if you want to monitorize or not 
%              the actuation voltages
%                 (Flag2 == YES)> monitorize voltages
%                 (Flag2 == NO)> omit monitorizing voltages

global Flag3
Flag3='YES'; % Secondary flag to set if you want to monitorize or not the 
%              actuation cable tensions and motor accelerations
%                 (Flag2 == YES)> monitorize cable tensions and accelerations
%                 (Flag2 == NO)> omit monitorizing cable tensions and accelerations

if strcmp(Flag00, 'YES')
        Flag4='YES';% Secondary flag to set if you want external PERTRUB. or not
%                 (Flag4 == YES)>  external pertrub. ENABLED
%                 (Flag4 == NO)> omit external pertrub.
end

tf = 15;    % Final simulation time
h = 0.001;  % Number of samples within final vectors (times and states)
opts = odeset('MaxStep',1e-3); % Max integration time step


% Reading data from file

m.g=9.8;    % Gravity constant [m/s^2] 

Geom_Vec=from_file('Geom_Data.txt');

m.ls=Geom_Vec(1,1);  % Length of the stick that links the ball with the platform,[m]  
m.mp=Geom_Vec(2,1);  % Mass of the platform [kg] 
m.mb=Geom_Vec(3,1);  % Mass of the ball+stick [kg]

m.ITpxx=Geom_Vec(4,1); %XX axis inercia of the platform [kg*m^2] 
m.ITpyy=Geom_Vec(5,1); %YY axis inercia of the platform [kg*m^2] 
m.ITpzz=Geom_Vec(6,1); %ZZ axis inercia of the platform [kg*m^2] 

m.ITbxx=Geom_Vec(7,1); %XX axis inercia of the platform [kg*m^2] 
m.ITbyy=Geom_Vec(8,1); %YY axis inercia of the platform [kg*m^2]
m.ITbzz=Geom_Vec(9,1); %ZZ axis inercia of the platform [kg*m^2] 


% A and B matrices of the hexapole geometry, all distances in [m]

m.B=1e-3*[0,-89.15,0;0,-89.15,0;77.21,44.57,0;77.21,44.57,0;-77.21,44.57,0; ...
                                                              -77.21,44.57,0]';
m.A=1e-3*[-231.62,-136.18,0;231.62,-136.18,0;233.74,-132.5,0;2.13,268.67,0; ...
                                             -2.13,268.67,0;-233.74,-132.5,0]';

                                         
% Defining motor related constants

m.ro=0.025/2; % Radius of the pulley fixed in MX-28 load axis [m]

m.Vs=12;      % Source Voltage [V]


m.Res=8.3;   % Resistance [om]
m.L=2.03e-3; % Inductance [H]
m.eff=0.836; % Gear efficiency


m.N=193;        % Gearbox ratio
m.Kom=93.0959;  % Motor speed constant [rad/V] 
m.Kt=0.0107;    % Torque constant [N*m/A] 
m.Jm=8.68e-8;   % Inertia value at motor axis [kg*m^2]
m.Jl=2.896e-7;  % Inercia volue at load axis [kg*m^2]
m.bm=8.87e-8;   % Friction value at motor axis [N*m*s]
m.EN=m.N*m.eff; % Constant that relates tauload with taumotor


% Equation of MX-28 constants

m.a=(m.EN*m.Kt)/m.Res; 
m.b=-(m.EN*(m.Kt)^2*m.N)/m.Res-m.EN*m.bm*m.N; 
m.c=-m.EN*m.N*m.Jm; 


% Vector of state in equilibrium position

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


% Cable lenghts

 m.lc=IKsolve(xstar,m);

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculating our utstar (the tensions of the cables that are in
% our home state)
wrenchstar=[0,0,(m.mp+m.mb)*m.g,0,0,0]';

% Calculating Jscrew home position
Jscrewstar=Calc_Jscrew_HexaMX28(xstar);
% Defining utaustar vector
utstar=inv(Jscrewstar)*wrenchstar;
utaustar=(m.ro)*utstar;

% Now we build voltages in our home position vector
uvstar=(m.Res/(m.Kt*m.EN))*utaustar;
disp(uvstar)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bryson Rule

CTE1=1;    % Constant to adjust position terms of Q matrix
CTE2=1;    % Constant to adjust velocity terms of Q matrix
CTE3=1100; % Constant to adjust actuation terms of R matrix

Q1= [ 10/(deg2rad(60))^2*eye(6),  zeros(6,2)                ; ...
         zeros(2,6)           ,  1/(deg2rad(5))^2 * eye(2) ];


Q2= [ (1/50)*((1/((5.6)^2))*eye(6)) ,   zeros(6,2)                ; ...
         zeros(2,6)               ,   (1/10)*(1/(deg2rad(5))^2 * eye(2)) ];


Q = [ CTE1*Q1,          zeros(8,8)   ; ...
      zeros(8,8),  CTE2*Q2      ];

R=CTE3*(1/(10)^2)*eye(6);

xthetastar(1:8,1)=[0,0,0,0,0,0,xstar(7),xstar(8)]';
xthetastar(9:16,1)=zeros(8,1);

[Alin,Blin]=Lineal_HexaMX28_Analitic( xstar, uvstar, m);

[K,SolRic,Eigvals]=lqr(Alin,Blin,Q,R);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R1 = @(x) [1 0 0; 0 cos(x(4)) -sin(x(4)); 0 sin(x(4)) cos(x(4));]; % Rot_x(psi)
R2 = @(x) [cos(x(5)) 0 sin(x(5)); 0 1 0; -sin(x(5)) 0 cos(x(5));]; % Rot_y(theta)
R3 = @(x) [cos(x(6)) -sin(x(6)) 0; sin(x(6)) cos(x(6)) 0; 0 0 1;]; % Rot_z(phi)
R4 = @(x) [1 0 0; 0 cos(x(7)) -sin(x(7)); 0 sin(x(7)) cos(x(7));]; % Rot_x(beta1)
R5 = @(x) [cos(x(8)) 0 sin(x(8)); 0 1 0; -sin(x(8)) 0 cos(x(8));]; % Rot_t(beta2)


        
switch Flag
    case 'ODE45'
        
         % Simulate with ode45 
         
         
         % Initial conditions
         
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
         
         switch Flag00
             case 'NO'
                 dxdt= @(t,xtheta) xdot_HexaMX28(t, xtheta, m, uvstar);
                 [times,states]=ode45(dxdt,t,xtheta0,opts);
             case 'YES'
                 if strcmp(Flag4, 'YES')
                     dxdt= @(t,xtheta) xdot_HexaMX28(t, xtheta, m, uvstar, ...
                         xthetastar, K, @uo_pert_wrench_HexaMX28);
                     [times,states]=ode45(dxdt,t,xtheta0,opts); 
                 else
                     dxdt= @(t,xtheta) xdot_HexaMX28(t, xtheta, m, uvstar, ...
                         xthetastar, K);
                     [times,states]=ode45(dxdt,t,xtheta0,opts); 
                 end
             otherwise
                 disp('this Flag00 does not exist');
         end

         switch Flag01
             case 'FULL'
                 figure;
                 plot(times,rad2deg(states(:,1)))
                 xlabel('t(s)'),ylabel('theta1(t) [deg]'),title('State')

                 figure;
                 plot(times,rad2deg(states(:,2)))
                 xlabel('t(s)'),ylabel('theta2(t) [deg]'),title('State')

                 figure;
                 plot(times,rad2deg(states(:,3)))
                 xlabel('t(s)'),ylabel('theta3(t) [deg]'),title('State')

                 figure;
                 plot(times,rad2deg(states(:,4)))
                 xlabel('t(s)'),ylabel('theta4(t) [deg]'),title('State')

                 figure;
                 plot(times,rad2deg(states(:,5)))
                 xlabel('t(s)'),ylabel('theta5(t) [deg]'),title('State')

                 figure;
                 plot(times,rad2deg(states(:,6)))
                 xlabel('t(s)'),ylabel('theta6(t) [deg]'),title('State')

                 figure;
                 plot(times,rad2deg(states(:,7)))
                 xlabel('t(s)'),ylabel('beta1(t) [deg]'),title('State')

                 figure;
                 plot(times,rad2deg(states(:,8)))
                 xlabel('t(s)'),ylabel('beta2(t) [deg]'),title('State')

                 figure;
                 plot(times,states(:,9))
                 xlabel('t(s)'),ylabel('theta1dot(t)'),title('State')

                 figure;
                 plot(times,states(:,10))
                 xlabel('t(s)'),ylabel('theta2dot(t)'),title('State')

                 figure;
                 plot(times,states(:,11))
                 xlabel('t(s)'),ylabel('theta3dot(t)'),title('State')

                 figure;
                 plot(times,states(:,12))
                 xlabel('t(s)'),ylabel('theta4dot(t)'),title('State')

                 figure;
                 plot(times,states(:,13))
                 xlabel('t(s)'),ylabel('theta5dot(t)'),title('State')

                 figure;
                 plot(times,states(:,14))
                 xlabel('t(s)'),ylabel('theta6dot(t)'),title('State')

                 figure;
                 plot(times,states(:,15))
                 xlabel('t(s)'),ylabel('beta1dot(t)'),title('State')

                 figure;
                 plot(times,states(:,16))
                 xlabel('t(s)'),ylabel('beta2dot(t)'),title('State')
                 
             case 'PACK'
                 figure; 
                 box on
                 hold on
                 plot(times,rad2deg(states(:,1)),'LineWidth',3) 
                 plot(times,rad2deg(states(:,2)),'LineWidth',3)
                 plot(times,rad2deg(states(:,3)),'LineWidth',3)
                 plot(times,rad2deg(states(:,4)),'LineWidth',3)
                 plot(times,rad2deg(states(:,5)),'LineWidth',3)
                 plot(times,rad2deg(states(:,6)),'LineWidth',3)
                 
                 xlabel('t(s)'),ylabel('theta1-theta6 [deg]'), ... 
                                              title('Motor Angles')
                 hold off
 
                 figure;
                 box on
                 hold on
                 plot(times,rad2deg(states(:,7)),'r','LineWidth',3)
                 plot(times,rad2deg(states(:,8)),'g','LineWidth',3)
                 xlabel('t(s)'),ylabel('beta1=red beta2=green [deg]'), ... 
                                                   title('Pendulum Angles')
                 hold off
                 
                 figure;
                 box on
                 hold on
                 plot(times,states(:,9),'LineWidth',3)
                 plot(times,states(:,10),'LineWidth',3)
                 plot(times,states(:,11),'LineWidth',3)
                 plot(times,states(:,12),'LineWidth',3)
                 plot(times,states(:,13),'LineWidth',3)
                 plot(times,states(:,14),'LineWidth',3)
                 xlabel('t(s)'),ylabel('theta1dot-theta6dot [rad/s]'), ... 
                                                      title('Motor Speeds')
                 hold off

                 figure;
                 box on
                 hold on
                 plot(times,states(:,15),'r','LineWidth',3)
                 plot(times,states(:,16),'g','LineWidth',3)
                 xlabel('t(s)'),ylabel('beta1dot=red beta2dot=green [rad/s]' ...
                                                      ),title('Pendulum Speeds')
                 hold off
             otherwise
                 disp('this Flag01 does not exist')
         end
         
         
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
         switch Flag1
             
             case 'YES'
                 
                % Building statesx 
                statesx(1,:)=xtheta_to_xq(states(1,:)',xstar,m)';
                for i=2:max(size(states))
                    statesx(i,:)=xtheta_to_xq(states(i,:)',statesx(i-1,:)',m)';
                end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                switch Flag01
                    case 'FULL'
                        figure;
                        plot(times,statesx(:,1))
                        xlabel('t(s)'),ylabel('x(t)'),title('State')

                        figure;
                        plot(times,statesx(:,2))
                        xlabel('t(s)'),ylabel('y(t)'),title('State')

                        figure;
                        plot(times,statesx(:,3))
                        xlabel('t(s)'),ylabel('z(t)'),title('State')

                        figure;
                        plot(times,rad2deg(statesx(:,4)))
                        xlabel('t(s)'),ylabel('psi(t) [deg]'),title('State')

                        figure;
                        plot(times,rad2deg(statesx(:,5)))
                        xlabel('t(s)'),ylabel('theta(t) [deg]'),title('State')

                        figure;
                        plot(times,rad2deg(statesx(:,6)))
                        xlabel('t(s)'),ylabel('phi(t) [deg]'),title('State')

                        figure;
                        plot(times,statesx(:,9))
                        xlabel('t(s)'),ylabel('xdot(t)'),title('State')

                        figure;
                        plot(times,statesx(:,10))
                        xlabel('t(s)'),ylabel('ydot(t)'),title('State')

                        figure;
                        plot(times,statesx(:,11))
                        xlabel('t(s)'),ylabel('zdot(t)'),title('State')

                        figure;
                        plot(times,statesx(:,12))
                        xlabel('t(s)'),ylabel('psidot(t)'),title('State')

                        figure;
                        plot(times,statesx(:,13))
                        xlabel('t(s)'),ylabel('thetadot(t)'),title('State')

                        figure;
                        plot(times,statesx(:,14))
                        xlabel('t(s)'),ylabel('phidot(t)'),title('State')
                        
                    case 'PACK'
                        figure;
                        box on
                        hold on
                        plot(times,statesx(:,1),'r','LineWidth',3)
                        plot(times,statesx(:,2),'g','LineWidth',3)
                        plot(times,statesx(:,3),'b','LineWidth',3)
                        xlabel('t(s)'),ylabel('x=red y=green z=blue [m]'), ... 
                                                     title('Platform position')
                        hold off
                        
                        
                        figure;
                        box on
                        hold on
                        plot(times,rad2deg(statesx(:,4)),'r','LineWidth',3)
                        plot(times,rad2deg(statesx(:,5)),'b','LineWidth',3)
                        plot(times,rad2deg(statesx(:,6)),'g','LineWidth',3)
                        xlabel('t(s)'), ... 
                            ylabel('psi=red theta=blue phi=green [deg]'), ... 
                                                  title('Platform orientation')
                        hold off
                        
                        figure;
                        box on
                        hold on
                        plot(times,statesx(:,9),'r','LineWidth',3)
                        plot(times,statesx(:,10),'b','LineWidth',3)
                        plot(times,statesx(:,11),'g','LineWidth',3)
                        xlabel('t(s)'), ... 
                            ylabel('xdot=red ydot=blue zdot=green [m/s]'), ... 
                                                        title('Platform speed')
                        hold off
                        
                        figure;
                        box on
                        hold on
                        plot(times,statesx(:,12),'r','LineWidth',3)
                        plot(times,statesx(:,13),'b','LineWidth',3)
                        plot(times,statesx(:,14),'g','LineWidth',3)
                        xlabel('t(s)') ...
                            ,ylabel('psidot=red thetadot=blue phidot=green [rad/s]' ...
                                              ),title('Platform rotation speed')
                        hold off
                        
                    otherwise
                        disp('this Flag01 does not exist')
                end


                     %%Ploting Gtot
                     figure;
                     box on
                     hold on
                     Gp(3,max(size(statesx)))=0;
                     Gb(3,max(size(statesx)))=0;
                     Gtot(3,max(size(statesx)))=0;
                     for i=1:max(size(statesx))
                         Gp(1,i)=statesx(i,1);
                         Gp(2,i)=statesx(i,2);
                         Gp(3,i)=statesx(i,3);
                     end
                     for i=1:max(size(Gp))
                         R4i=R4(statesx(i,:));
                         R5i=R5(statesx(i,:));
                         Gb(:,i)=Gp(:,i)+R4i*R5i*([0, 0, m.ls]');
                     end
                     for i=1:max(size(Gp))
                         Gtot(:,i)=1/(m.mp+m.mb)*(m.mp*Gp(:,i)+m.mb*Gb(:,i));
                     end
                     plot(times,Gtot(1,:),'r','LineWidth',3)
                     plot(times,Gtot(2,:),'b','LineWidth',3)
                     plot(times,Gtot(3,:),'g','LineWidth',3)
                     xlabel('t(s)'),ylabel('Gtot x=red,y=blue,z=green [m]'), ... 
                                                          title('Gtot position')
                     hold off
             otherwise
                 disp('this input Flag1 does not exist')
         end
         
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
         switch Flag2
             case 'YES'
                 uvArray(6,max(size(times)))=0;
                 for i=1:max(size(times))
                     uvArray(:,i)=u_function_HexaMX28(times(i),states(i,:)', ... 
                                                         m,uvstar,xthetastar,K);
                 end   
                 figure;
                 box on
                 hold on
                 plot(times,uvArray(1,:),'r','LineWidth',3)
                 plot(times,uvArray(2,:),'b','LineWidth',3)
                 plot(times,uvArray(3,:),'g','LineWidth',3)
                 plot(times,uvArray(4,:),'y','LineWidth',3)
                 plot(times,uvArray(5,:),'k','LineWidth',3)
                 plot(times,uvArray(6,:),'m','LineWidth',3)
                 xlabel('t(s)'),ylabel('Voltage [V]'), ... 
                              title('Monitorizing Voltages')
                 hold off
             case 'NO'
                 disp('omitting monitorizing of wrenches')
             otherwise
                 disp('this input Flag2 does not exist')
         end
         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
         %% Monitorize cable tensions
         switch Flag3
             case'YES'
                 
                 xdotts=timeseries(xdotArray,tvec);
                 xdotresample=resample(xdotts,times);
                 xdot_Array=xdotresample.Data;
                 uCTArray(6,max(size(times)))=0;
                 
                 for i=1:max(size(times))
                     
                     uCTArray(:,i)=(1/m.ro)*(m.a*uvArray(:,i)+m.b* ... 
                         xdot_Array(1:6,1,i)+m.c*xdot_Array(9:14,1,i));
                 end
                 
                 figure;
                 box on
                 hold on
                 plot(times,uCTArray(1,:),'r','LineWidth',3)
                 plot(times,uCTArray(2,:),'b','LineWidth',3)
                 plot(times,uCTArray(3,:),'g','LineWidth',3)
                 plot(times,uCTArray(4,:),'y','LineWidth',3)
                 plot(times,uCTArray(5,:),'k','LineWidth',3)
                 plot(times,uCTArray(6,:),'m','LineWidth',3)
                 xlabel('t(s)'),ylabel('Cable tensions [N]'), ... 
                                          title('Monitorizing CT')
                 hold off
                 
                 % Monitorize accelerations
                 for i=1:max(size(times))
                     ACC_Array(:,i)=xdot_Array(9:16,1,i);
                 end
                 
                 
                 figure;
                 box on
                 hold on
                 plot(times,ACC_Array(1,:),'r','LineWidth',3)
                 plot(times,ACC_Array(2,:),'b','LineWidth',3)
                 plot(times,ACC_Array(3,:),'g','LineWidth',3)
                 plot(times,ACC_Array(4,:),'y','LineWidth',3)
                 plot(times,ACC_Array(5,:),'k','LineWidth',3)
                 plot(times,ACC_Array(6,:),'m','LineWidth',3)
                 xlabel('t(s)'),ylabel('theta1dotdot-theta6dotdot [rad/s^2]'), ... 
                                                     title('Motor Accelerations')
                 hold off
                 
                 figure;
                 box on
                 hold on
                 plot(times,ACC_Array(7,:),'r','LineWidth',3)
                 plot(times,ACC_Array(8,:),'g','LineWidth',3)
                 xlabel('t(s)'), ... 
                     ylabel('beta1dotdot=red beta2dotdot=green [rad/s^2]'), ... 
                                                 title('Pendulum Accelerations')
                 hold off
                 
             case 'NO'
                 disp('omitting monitorizing of cable tenisons')
             otherwise
                 disp('this input Flag3 does not exist')
         end
         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    case 'FinDif'
        
        xthetastar(1:8,1)=[0,0,0,0,0,0,xstar(7),xstar(8)]';
        xthetastar(9:16,1)=zeros(8,1);

        display(xthetastar);
        display(utaustar);
        display(xstar);
        display(utstar);
        display(uvstar);

        [Alin,Blin]=Lineal_HexaMX28_Analitic( xstar, uvstar, m);
        
        [K,SolRic,Eigvals]=lqr(Alin,Blin,Q,R);
        
  
    case 'VIDEO'   
        
        time=cputime;
         % Animation
         % h is the sampling time 
         % n is the scaling factor in order not to plot with the same step
         % than during the integration with ode45
         fs=30;
         n=round(1/(fs*h));
         
        % Set up the movie.
        writerObj = VideoWriter('hexapole_HexaMX28','MPEG-4'); % Name it.
        % writerObj.FileFormat = 'mp4';
        writerObj.FrameRate = fs; % How many frames per second.
        open(writerObj);

         for i = 1:n:length(times)
                    q.x=statesx(i,1);
                    q.y=statesx(i,2);
                    q.z=statesx(i,3);
                    q.psi=statesx(i,4);
                    q.theta=statesx(i,5);
                    q.phi=statesx(i,6);
                    q.beta1=statesx(i,7);
                    q.beta2=statesx(i,8);
                    
                    elapsed = cputime-time;
                    if elapsed>200
                        disp(elapsed);
                        disp('took too long to generate the video')
                        break
                    else
                        % 'gcf' can handle if you zoom in to take a movie.
                        
                        Draw_DM_HexaMX28(m,q)
                        frame = getframe(gcf); 
                        writeVideo(writerObj, frame); 
                    end
                    
         end
         close(writerObj); % Saves the movie.
     
     case 'NEW_VIDEO' 

            time=cputime;
             % Animation
             % h is the sampling time 
             % n is the scaling factor in order not to plot with the same step
             % than during the integration with ode45
             fs=30;
             n=round(1/(fs*h));

            % Set up the movie.
            writerObj = VideoWriter('hexapole_STL','MPEG-4'); % Name it.
            %writerObj.FileFormat = 'mp4';
            writerObj.FrameRate = fs; % How many frames per second.
            open(writerObj);


             %draw external structure
             f100=Draw_STL();

             for i = 1:n:length(times)
                        q.x=statesx(i,1);
                        q.y=statesx(i,2);
                        q.z=statesx(i,3);
                        q.psi=statesx(i,4);
                        q.theta=statesx(i,5);
                        q.phi=statesx(i,6);
                        q.beta1=statesx(i,7);
                        q.beta2=statesx(i,8);

                        elapsed = cputime-time;
                        if elapsed>400
                            disp(elapsed);
                            disp('took too long to generate the video')
                            break

                        else
                            % 'gcf' can handle if you zoom in to take a movie.
                            [ Array_plot ]=Draw_mobile(m,q);
                            frame = getframe(gcf); 
                            writeVideo(writerObj, frame);
                            Delete_plots(Array_plot);
                        end

             end
             close(writerObj); % Saves the movie.
         
    otherwise
        disp('This flag input does not exist');
end