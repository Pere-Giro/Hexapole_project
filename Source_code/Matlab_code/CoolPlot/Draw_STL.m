%% Draw_STL


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% This function plots the octoedrical structure. In this case we use the stl 
% model obtained with SOLIDWORKS


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% no inputs needed

%%%%%%%%%%%%%%%%%%%%%% Outputs %%%%%%%%%%%%%%%%%%%%%%

% plot of the octaedrical structure using stl model

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f100] = Draw_STL( )

f100=figure(100);
pos_fig1 = [0 0 1280 720];
set(gcf,'Position',pos_fig1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Loads the UNIONS and translates and rotates it to its place

[univ,unif] = stlRead('nomes unions.stl');
univ2 = [univ(:,1:3)'; ones(size(univ(:,1)))'];

% Warning my laptop function rotx is aleady in deg 
univ3 = (transl(-300, 300, -408)*trotx(90)*univ2)'; %(-300,300,-408) 
univ4 = univ3(:,1:3);

% Loads BARS and translates and rotates it to its place
[barsv,barsf] = stlRead('barrascarbono.stl');
barsv2 = [barsv(:,1:3)'; ones(size(barsv(:,1)))'];

% Warning my laptop function rotx is aleady in deg 
barsv3 = (transl(-300, 290, -400)*trotx(90)*barsv2)'; %(-300,300,-408) 
barsv4 = barsv3(:,1:3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
grid on;
trplot(eye(3), 'length', 100, 'color', 'k');

hold on
axis([-400 400 -400 400 -500 700]);
daspect([1 1 1]);
view(3); 
camlight;
lighting gouraud;

%draw bars
patch('Faces',barsf,'Vertices',barsv4,'FaceColor', (1/255)*[89 89 89], ...
    'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);


%draw unions
patch('Faces',unif,'Vertices',univ4,'FaceColor', (1/255)*[252 255 121], ...
    'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.15);


hold off

saveas(figure(100), 'temp.fig')


end

