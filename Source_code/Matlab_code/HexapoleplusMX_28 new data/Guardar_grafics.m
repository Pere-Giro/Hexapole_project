%% Seccio 1


CN=' 15sec simulation b1 1deg b2 1deg stick 0,8m new masses V6 pert.png';

x=('x');
y=('y');
z=('z');
psi=('psi');
theta=('theta');
phi=('phi');
beta1=('beta1');
beta2=('beta2');
xdot=('xdot');
ydot=('ydot');
zdot=('zdot');
psidot=('psidot');
thetadot=('thetadot');
phidot=('phidot');
beta1dot=('beta1dot');
beta2dot=('beta2dot');
Gtot=('Gtot');

switch Flag01
    case 'FULL'
        for i=1:16
            if i==1
                names='theta1';
            elseif i==2
                names='theta2';
            elseif i==3
                names='theta3';
            elseif i==4
                names='theta4';
            elseif i==5
                names='theta5';
            elseif i==6
                names='theta6';
            elseif i==7
                names='beta1';
            elseif i==8
                names='beta2';


            elseif i==9
                names='theta1dot';
            elseif i==10
                names='theta2dot';
            elseif i==11
                names='theta3dot';
            elseif i==12
                names='theta4dot';
            elseif i==13
                names='theta5dot';
            elseif i==14
                names='theta6dot';
            elseif i==15
                names='beta1dot';
            elseif i==16
                names='beta2dot';  
            end
            saveas(figure(i),strcat(names,CN));
        end



        switch Flag1
            case 'YES'

                for i=17:29
                    if i==17
                        names=x;
                    elseif i==18
                        names=y;
                    elseif i==19
                        names=z;
                    elseif i==20
                        names=psi;
                    elseif i==21
                        names=theta;
                    elseif i==22
                        names=phi;

                    elseif i==23
                        names=xdot;
                    elseif i==24
                        names=ydot;
                    elseif i==25
                        names=zdot;
                    elseif i==26
                        names=psidot;
                    elseif i==27
                        names=thetadot;
                    elseif i==28
                        names=phidot;
                    elseif i==29
                        names=Gtot;
                    else 
                        names='out of range';
                    end

                saveas(figure(i),strcat(names,CN));
                end
            otherwise
                disp('xq wont be saved')
        end
        
        
    case 'PACK'
        counter=4;
        for i=1:counter
            if i==1
                names='MotorAngels';
            elseif i==2
                names='PendulumAngels';
            elseif i==3
                names='MotorSpeeds';
            elseif i==4
                names='PendulumSpeeds';
            end
            saveas(figure(i),strcat(names,CN));
        end
                
        switch Flag1
            
            case 'YES'
                counter=counter+4;
                for i=5:counter
                    if i==5
                        names='PlatformPosition';
                    elseif i==6
                        names='PlatformOrientation';
                    elseif i==7
                        names='PlatformSpeed';
                    elseif i==8
                        names='PlatfromRotationSpeed';
                    end
                    saveas(figure(i),strcat(names,CN));
                end
                
            otherwise
                disp('xq states will not be saved')
        end
        
        %Saveing Gtot plot
        counter=counter+1;
        names='Gtot';
        saveas(figure(counter),strcat(names,CN));
        
        switch Flag2
            case 'YES'
                counter=counter+1;
                names='Voltages';
                saveas(figure(counter),strcat(names,CN));
            otherwise
                disp('Voltages will not be saved')
        end
        
        switch Flag3
            case 'YES'
                counter=counter+1;
                names='CableTensions';
                saveas(figure(counter),strcat(names,CN));
                
                counter=counter+1;
                names='Motor Accelerations';
                saveas(figure(counter),strcat(names,CN));
                
                counter=counter+1;
                names='Pendulum Accelerations';
                saveas(figure(counter),strcat(names,CN));
                
            otherwise
                disp('cable tensions will not be saved')
        end
    otherwise
        disp('this Flag01 does not exist')
end

                
        
                        
            
            
        




