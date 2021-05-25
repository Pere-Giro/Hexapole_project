%% Makeing a video of Simulink results
%Warning Plots_form Simulink must be executed before this script with
%Simulink_Flag set to 'YES'
        
h=deltatime;
time=cputime;
% Animation
% h is the sampling time 
% n is the scaling factor in order not to plot with the same step
% than during the integration with ode45
fs=30;
n=round(1/(fs*h));

% Set up the movie.
writerObj = VideoWriter('Hexapole_Simulink','MPEG-4'); % Name it.
%writerObj.FileFormat = 'mp4';
writerObj.FrameRate = fs; % How many frames per second.
open(writerObj);

for i = 1:n:max(size(statesx))
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
        Draw_DM_HexaMX28(m,q)
        frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
        writeVideo(writerObj, frame); 
    end

end
close(writerObj); % Saves the movie.