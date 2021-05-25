%% Plotting xtheta

Simulink_Flag='YES';%Flag to define what you want the script to do
%                 (Simulink_Flag == YES)> monitorize x
%                 (Simulink_Flag == NO)> omit monitorizing of x


Sim_T=4; %simulation time in seconds

Array_Simulink1=get(xtheta_states_Array,'Data');

maxind=max(size(Array_Simulink1));
states(maxind,16)=0;
for i=1:maxind
    states(i,:)=Array_Simulink1(:,1,i)';
end

deltatime=Sim_T/maxind;

times(1,maxind)=0;
times(1,1)=0;
for i=2:maxind
    times(1,i)=times(1,i-1)+deltatime;
end


figure; 
hold on
plot(times,rad2deg(states(:,1))) 
plot(times,rad2deg(states(:,2))) 
plot(times,rad2deg(states(:,3))) 
plot(times,rad2deg(states(:,4)))
plot(times,rad2deg(states(:,5)))
plot(times,rad2deg(states(:,6)))

xlabel('t(s)'),ylabel('theta1-theta6 [deg]'),title('Motor Angles')
hold off

figure;
hold on
plot(times,rad2deg(states(:,7)),'r')
plot(times,rad2deg(states(:,8)),'g')
xlabel('t(s)'),ylabel('beta1=red beta2=green [deg]'),title('Pendulum Angles')
hold off

figure;
hold on
plot(times,states(:,9))
plot(times,states(:,10))
plot(times,states(:,11))
plot(times,states(:,12))
plot(times,states(:,13))
plot(times,states(:,14))
xlabel('t(s)'),ylabel('theta1dot-theta6dot [rad/s]'),title('Motor Speeds')
hold off

figure;
hold on
plot(times,states(:,15),'r')
plot(times,states(:,16),'g')
xlabel('t(s)'),ylabel('beta1dot=red beta2dot=green [rad/s]'),title('Pendulum Speeds')
hold off

switch Simulink_Flag
    case 'YES'

        %building statesx 
        statesx(1,:)=xtheta_to_xq(states(1,:)',xstar,m)';
        for i=2:max(size(states))
            statesx(i,:)=xtheta_to_xq(states(i,:)',statesx(i-1,:)',m)';
        end

        figure;
        hold on
        plot(times,statesx(:,1),'r')
        plot(times,statesx(:,2),'g')
        plot(times,statesx(:,3),'b')
        xlabel('t(s)'),ylabel('x=red y=green z=blue [m]'),title('Platform position')
        hold off


        figure;
        hold on
        plot(times,rad2deg(statesx(:,4)),'r')
        plot(times,rad2deg(statesx(:,5)),'b')
        plot(times,rad2deg(statesx(:,6)),'g')
        xlabel('t(s)'),ylabel('psi=red theta=blue phi=green [deg]'),title('Platform orientation')
        hold off

        figure;
        hold on
        plot(times,statesx(:,9),'r')
        plot(times,statesx(:,10),'b')
        plot(times,statesx(:,11),'g')
        xlabel('t(s)'),ylabel('xdot=red ydot=blue zdot=green [m/s]'),title('Platform speed')
        hold off

        figure;
        hold on
        plot(times,statesx(:,12),'r')
        plot(times,statesx(:,13),'b')
        plot(times,statesx(:,14),'g')
        xlabel('t(s)'),ylabel('psidot=red thetadot=blue phidot=green [rad/s]'),title('Platform rotation speed')
        hold off

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
        plot(times,Gtot(1,:),'r')
        plot(times,Gtot(2,:),'b')
        plot(times,Gtot(3,:),'g')
        xlabel('t(s)'),ylabel('Gtot x=red,y=blue,z=green [m]'),title('Gtot position')
        hold off
    otherwise
        disp('omitting x state plot')
end

