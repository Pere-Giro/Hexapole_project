%% Secction 1 just a test with the new method of plotting
%%Loads the spiral model and translates and rotates it to its place
[hexav,hexaf] = stlRead('hexacrane.stl');
hexav2 = [hexav(:,1:3)'; ones(size(hexav(:,1)))'];
hexav3 = (transl(0, 0, 0)*trotx(deg2rad(90))*hexav2)'; %(-300,300,-408)
hexav4 = hexav3(:,1:3);




figure(2);

grid on;
trplot(eye(3), 'length', 100, 'color', 'k');

hold on
axis([-400 400 -400 400 -500 200]);
daspect([1 1 1]);
view(3); 
camlight;
lighting gouraud;



patch('Faces',hexaf,'Vertices',hexav4,'FaceColor', [0.8 0.8 1.0], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);
hold off;

%% use draw_STL function to plot a frame

global f100

q.x=0;
q.y=0;
q.z=-0.3;
q.psi=0;
q.theta=0;
q.phi=0;
q.beta1=0;
q.beta2=0;

m.ls=0.96724; 
m.B=1e-3*[0,-89.15,0;0,-89.15,0;77.21,44.57,0;77.21,44.57,0;-77.21,44.57,0;-77.21,44.57,0]';
m.A=1e-3*[-231.62,-136.18,0;231.62,-136.18,0;233.74,-132.5,0;2.13,268.67,0;-2.13,268.67,0;-233.74,-132.5,0]';
        
Draw_STL(); %this function has been split in two Draw_STL & Draw_mobile

%% test to copy figures; (this is the best way to do it)



figure(102);
pos_fig1 = [0 0 1280 720];
set(gcf,'Position',pos_fig1);
f102=gcf;
% hello=findobj(figure(100));
% copyobj(hello,h2);
hexa=get(f100,'children');
copyobj(hexa,f102);



        