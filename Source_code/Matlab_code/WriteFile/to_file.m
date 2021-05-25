%% to_file


%%%%%%%%%%%%%%%%%%%%%% Purpose %%%%%%%%%%%%%%%%%%%%%% 

% This function writes a .txt file containig the parameters used
% to build the model from a vector named Geom_Vector


%%%%%%%%%%%%%%%%%%%%%% Inputs %%%%%%%%%%%%%%%%%%%%%% 

% Parameters of the model

% nomfitxer   > name the .txt file you want to create
% Geom_Vector > vector containig the parameters of the model


%%%%%%%%%%%%%%%%%%%%%% Outputs %%%%%%%%%%%%%%%%%%%%%% 

% nomfitxer.txt

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ Array ] = to_file( nomfitxer,Geom_Vector )

fid=fopen(strcat(nomfitxer,'.txt'),'w');
for i=1:max(size(Geom_Vector))
    fprintf(fid,num2str(Geom_Vector(i),16));
    fprintf(fid,'\t\t');
end
fprintf(fid,'\n');

fclose(fid);

Array=Geom_Vector;

end

