% Definicio de constants
%variable to define if you want the animation or not

Flag='lsim';%Flag to define what you want the script to do 
%               (Flag == lsim)> run the simulation with lsim
%               (Flag == VIDEO)> output a video of an animation of the dynamic model
%               (Flag ==FinDif)> output AlinF and BlinF matrices to build linealized model through FinDif
Flag1='YES';%Secondary flag to set if you want to monitorize or not x state (q domain)
%                 (Flag2 == YES)> monitorize x
%                 (Flag2 == NO)> omit monitorizing of x
Flag2='NO';%Secondary flag to set if you want to monitorize or not the actuation cable tensions
%                 (Flag2 == YES)> monitorize cable tensions
%                 (Flag2 == NO)> omit monitorizing of cable tensions
                                  
                                                                                                                    
        
        tf = 4;    % final simulation time
        h = 0.01; %number of samples within final vectors (times and states)
        opts = odeset('MaxStep',1e-2); % max integration time step
        
        m.g=9.8;    % gravity constant [m/s^2] **(used to plot and draw only//main calculus in MAPLE)**
        
        m.ls=0.96724;  % length of the stick that links the ball with the platform,[m]  **(used to plot and draw only//main calculus in MAPLE)**
        m.mp=0.59895;     % mass of the platform [kg] **(used to plot and draw only//main calculus in MAPLE)**
        m.mb=0.41472;   % mass of the ball+stick [kg] **(used to plot and draw only//main calculus in MAPLE)**
        
        m.ITpxx=270725.71*1e-9; %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        m.ITpyy=270725.71*1e-9; %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        m.ITpzz=409645.04*1e-9; %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        
        m.ITbxx=11262205.59*1e-9; %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        m.ITbyy=11262205.59*1e-9; %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        m.ITbzz=77397.82*1e-9; %XX axis inercia of the platform [kg*m^2] **(used to plot and draw only//main calculus in MAPLE)**
        
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

        
        
        
 %defining xstar vector 
        xstar(1)=0;
        xstar(2)=0;
        xstar(3)=-0.300;
        xstar(4)=0;
        xstar(5)=0;
        xstar(6)=0;
        xstar(7)=0;
        xstar(8)=0;
        
        xstar(9)=0;
        xstar(10)=0;
        xstar(11)=0;
        xstar(12)=0;
        xstar(13)=0;
        xstar(14)=0;
        xstar(15)=0;
        xstar(16)=0;
        
        if isrow(xstar)
            xstar=xstar';
        else
            xstar=xstar;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         % Calculating our utstar (the tensions of the cables that are in
         % our home state)
        wrenchstar=[0,0,(m.mp+m.mb)*m.g,0,0,0]';
        
        %calculating Jscrew home position
        Jscrewstar=Calc_Jscrew_MXcord(xstar);
        %defining ustar vector
        utstar=inv(Jscrewstar)*wrenchstar;
        utaustar=(m.ro)*utstar;
        disp(utaustar)
        
       
        %%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Bryson rule
% %         maxrangxy=0.12;
% %         maxrangz=0.25;
% %         maxrangrpy=deg2rad(10);
% %         maxrangb1b2=deg2rad(10);
% %         
% %         maxrangu=(m.ro)*(6.5-mean(utstar));
% %         
% %         GainXY=1/(maxrangxy)^2;
% %         GainZ=1/(maxrangz)^2;
% %         Gainrpy=1/(maxrangrpy)^2;
% %         Gainb1b2=1/(maxrangb1b2)^2;
% %         Gainu=1/(maxrangu)^2;
% %         
% %         GainXYdot=GainXY/10;
% %         GainZdot=GainZ/10;
% %         Gainrpydot=Gainrpy/10;
% %         Gainb1b2dot=Gainb1b2/10;
% %         
% %         
% %         
% %         Qq=[GainXY*eye(2),zeros(2,6);zeros(1,2),GainZ,zeros(1,5);zeros(3,3),Gainrpy*eye(3),zeros(3,2);zeros(2,6),Gainb1b2*eye(2)];
% %         Qqdot=[GainXYdot*eye(2),zeros(2,6);zeros(1,2),GainZdot,zeros(1,5);zeros(3,3),Gainrpydot*eye(3),zeros(3,2);zeros(2,6),Gainb1b2dot*eye(2)];
% %         Q=[Qq,zeros(8);zeros(8),Qqdot];
% %         R=Gainu*eye(6);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        Q1=[1/(deg2rad(360))^2*eye(6),zeros(6,2);zeros(2,6),1/(deg2rad(5))^2*eye(2)];
        Q2=1/10*Q1;
        
        Q=[Q1,zeros(8,8);zeros(8,8),Q2];
        R=1/(6.5*m.ro-utaustar(1))^2*eye(6);
        
        xthetastar(1:8)=[0,0,0,0,0,0,xstar(7),xstar(8)]';
        xthetastar(9:16)=zeros(8,1);
        
        if isrow(xthetastar)
            xthetastar=xthetastar';
        else
            xthetastar=xthetastar;
        end

        [Alin,Blin]=Lineal_HexaMxcord_Analitic( xstar, utstar, utaustar, m, 'JEtauG');

        [K,SolRic,Eigvals]=lqr(Alin,Blin,Q,R);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%


        %Note: X (state vector) is 10th dimensional column vector, built
        %with q and qdot
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
R1 = @(x) [1 0 0; 0 cos(x(4)) -sin(x(4)); 0 sin(x(4)) cos(x(4));]; % Rot_x(psi)
R2 = @(x) [cos(x(5)) 0 sin(x(5)); 0 1 0; -sin(x(5)) 0 cos(x(5));]; % Rot_y(theta)
R3 = @(x) [cos(x(6)) -sin(x(6)) 0; sin(x(6)) cos(x(6)) 0; 0 0 1;]; % Rot_z(phi)
R4 = @(x) [1 0 0; 0 cos(x(7)) -sin(x(7)); 0 sin(x(7)) cos(x(7));]; % Rot_x(beta1)
R5 = @(x) [cos(x(8)) 0 sin(x(8)); 0 1 0; -sin(x(8)) 0 cos(x(8));]; % Rot_t(beta2)

        
        
switch Flag
    case 'lsim'
        
   %Simulate with ode45 
         
         %initial conditions
         x0=[0,0,-0.3,0,0,0,deg2rad(3),0,0,0,0,0,0,0,0,0]';
        
         global xq
         xq=x0;

         % Time
         t = 0:h:tf;
         
         global xthetaplot
         xthetaplot(16,1)=0;
         
         global xbarrax
         xbarrax(16,1)=0;
        
%          global xq
%          xq=x0;
         
         global xqplot
         xqplot(3,1)=-0.3;
         xqplot(16,1)=0;
         
         
         xtheta0=x_to_xtheta(x0,m);
         display(xtheta0);
         pause(3);
         
        
         umatrix(max(size(t)),6)=0;
         Acl=(Alin-Blin*K);
         Blsim=zeros(16,6);
         Clsim=eye(16);
         Dlsim=zeros(16,6);
         sys=ss(Acl,Blsim,Clsim,Dlsim);
         
         tic;
         [ylsim,tlsim,xlsim]=lsim(sys,umatrix,t,xtheta0); 
         toc;
         
         
         
         
         
         figure;
         plot(tlsim,xlsim(:,1))
         xlabel('t(s)'),ylabel('theta1(t)'),title('State')
         
         figure;
         plot(tlsim,xlsim(:,2))
         xlabel('t(s)'),ylabel('theta2(t)'),title('State')

         figure;
         plot(tlsim,xlsim(:,3))
         xlabel('t(s)'),ylabel('theta3(t)'),title('State')
         
         figure;
         plot(tlsim,xlsim(:,4))
         xlabel('t(s)'),ylabel('theta4(t)'),title('State')
         
         figure;
         plot(tlsim,xlsim(:,5))
         xlabel('t(s)'),ylabel('theta5(t)'),title('State')

         figure;
         plot(tlsim,xlsim(:,6))
         xlabel('t(s)'),ylabel('theta6(t)'),title('State')

         figure;
         plot(tlsim,rad2deg(xlsim(:,7)))
         xlabel('t(s)'),ylabel('beta1(t) [deg]'),title('State')

         figure;
         plot(tlsim,rad2deg(xlsim(:,8)))
         xlabel('t(s)'),ylabel('beta2(t) [deg]'),title('State')

         figure;
         plot(tlsim,xlsim(:,9))
         xlabel('t(s)'),ylabel('theta1dot(t)'),title('State')

         figure;
         plot(tlsim,xlsim(:,10))
         xlabel('t(s)'),ylabel('theta2dot(t)'),title('State')

         figure;
         plot(tlsim,xlsim(:,11))
         xlabel('t(s)'),ylabel('theta3dot(t)'),title('State')
         
         figure;
         plot(tlsim,xlsim(:,12))
         xlabel('t(s)'),ylabel('theta4dot(t)'),title('State')

         figure;
         plot(tlsim,xlsim(:,13))
         xlabel('t(s)'),ylabel('theta5dot(t)'),title('State')

         figure;
         plot(tlsim,xlsim(:,14))
         xlabel('t(s)'),ylabel('theta6dot(t)'),title('State')

         figure;
         plot(tlsim,xlsim(:,15))
         xlabel('t(s)'),ylabel('beta1dot(t)'),title('State')

         figure;
         plot(tlsim,xlsim(:,16))
         xlabel('t(s)'),ylabel('beta2dot(t)'),title('State')
         
         
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         switch Flag1
             case 'YES'
                     %building statesx 
                     statesx(1,:)=xtheta_to_xq(xlsim(1,:)',xstar,m)';
                     for i=2:max(size(xlsim))
                         statesx(i,:)=xtheta_to_xq(xlsim(i,:)',statesx(i-1,:)',m)';
                     end
                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                     figure;
                     plot(tlsim,statesx(:,1))
                     xlabel('t(s)'),ylabel('x(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,2))
                     xlabel('t(s)'),ylabel('y(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,3))
                     xlabel('t(s)'),ylabel('z(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,4))
                     xlabel('t(s)'),ylabel('psi(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,5))
                     xlabel('t(s)'),ylabel('theta(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,6))
                     xlabel('t(s)'),ylabel('phi(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,9))
                     xlabel('t(s)'),ylabel('xdot(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,10))
                     xlabel('t(s)'),ylabel('ydot(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,11))
                     xlabel('t(s)'),ylabel('zdot(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,12))
                     xlabel('t(s)'),ylabel('psidot(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,13))
                     xlabel('t(s)'),ylabel('thetadot(t)'),title('State')

                     figure;
                     plot(tlsim,statesx(:,14))
                     xlabel('t(s)'),ylabel('phidot(t)'),title('State')


                     %%Ploting Gtot
                     figure;
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
                     plot(tlsim,Gtot(1,:),'r')
                     plot(tlsim,Gtot(2,:),'b')
                     plot(tlsim,Gtot(3,:),'g')
                     xlabel('t(s)'),ylabel('Gtot [x=red,y=blue,z=green]'),title('Gtot position')
                     hold off
             otherwise
                 disp('this input Flag1 does not exist')
         end
         
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         
 
% %          %monitorize wrench
% %          switch Flag2
% %              case 'YES'
% %                  uArray(6,max(size(times)))=0;
% %                  for i=1:max(size(times))
% %                      uArray(:,i)=u_function_HexaMX_28(times(i),states(i,:)',ustar,xstar,K); %(times(i),states(i,:)',ustar,xstar,K)
% %                  end   
% %                  figure;
% %                  hold on
% %                  plot(times,uArray(1,:),'r')
% %                  plot(times,uArray(2,:),'b')
% %                  plot(times,uArray(3,:),'g')
% %                  plot(times,uArray(4,:),'y')
% %                  plot(times,uArray(5,:),'k')
% %                  plot(times,uArray(6,:),'m')
% %                  xlabel('t(s)'),ylabel('CableTensions'),title('Monitorizing cable tensions')
% %                  hold off
% %              case 'NO'
% %                  disp('omitting monitorizing of wrenches')
% %              otherwise
% %                  disp('this input Flag2 does not exist')
% %          end
% %          
% %         
         


    case 'FinDif'
        xthetastar(1:8)=[0,0,0,0,0,0,xstar(7),xstar(8)]';
        xthetastar(9:16)=zeros(8,1);
        
        if isrow(xthetastar)
            xthetastar=xthetastar';
        else
            xthetastar=xthetastar;
        end
        
        display(xthetastar);
        display(utaustar);
        display(xstar);
        display(utstar);
        
        [AlinF,BlinF]=Lineal_HexaMxcord_FinDif( xthetastar, utaustar, m, xstar );
        [Alin1,Blin1]=Lineal_HexaMxcord_Analitic( xstar, utstar, utaustar, m, 'JEuG');
        [Alin2,Blin2]=Lineal_HexaMxcord_Analitic( xstar, utstar, utaustar, m, 'JEtauG');
        
        [K0,SolRic0,Eigvals0]=lqr(AlinF,BlinF,Q,R);
        [K1,SolRic1,Eigvals1]=lqr(Alin1,Blin1,Q,R);
        [K2,SolRic2,Eigvals2]=lqr(Alin2,Blin2,Q,R);
        
        
         
         
        
    case 'VIDEO'   
        
        time=cputime;
         % Animation
         % h is the sampling time 
         % n is the scaling factor in order not to plot with the same step
         % than during the integration with ode45
         fs=30;
         n=round(1/(fs*h));
         
        % Set up the movie.
        writerObj = VideoWriter('hexapole2','MPEG-4'); % Name it.
        %writerObj.FileFormat = 'mp4';
        writerObj.FrameRate = fs; % How many frames per second.
        open(writerObj);

         
         
         
         for i = 1:n:length(tlsim)
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
                        Draw_DM_MXcord(m,q)
                        frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
                        writeVideo(writerObj, frame); 
                    end
                    
         end
         close(writerObj); % Saves the movie.
         
    otherwise
        disp('This flag input does not exist');
end

 