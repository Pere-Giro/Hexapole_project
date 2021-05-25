%% Seccio 1


CN=' Mxcord simulation tot test Mtheta and odestep cntrl law.png';

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

for i=2:17
    if i==2
        names='theta1';
    elseif i==3
        names='theta2';
    elseif i==4
        names='theta3';
    elseif i==5
        names='theta4';
    elseif i==6
        names='theta5';
    elseif i==7
        names='theta6';
    elseif i==8
        names='beta1';
    elseif i==9
        names='beta2';
        
        
    elseif i==10
        names='theta1dot';
    elseif i==11
        names='theta2dot';
    elseif i==12
        names='theta3dot';
    elseif i==13
        names='theta4dot';
    elseif i==14
        names='theta5dot';
    elseif i==15
        names='theta6dot';
    elseif i==16
        names='beta1dot';
    elseif i==17
        names='beta2dot';  
    end
    saveas(figure(i),strcat(names,CN));
end
  


switch Flag1
    case 'YES'
        
        for i=18:30
        if i==18
            names=x;
        elseif i==19
            names=y;
        elseif i==20
            names=z;
        elseif i==21
            names=psi;
        elseif i==22
            names=theta;
        elseif i==23
            names=phi;
     
        elseif i==24
            names=xdot;
        elseif i==25
            names=ydot;
        elseif i==26
            names=zdot;
        elseif i==27
            names=psidot;
        elseif i==28
            names=thetadot;
        elseif i==29
            names=phidot;
        elseif i==30
            names=Gtot;
        else 
            names='out of range'
        end

        saveas(figure(i),strcat(names,CN));
        end
    otherwise
        disp('xq wont be saved')
end




