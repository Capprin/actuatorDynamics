function actuatorAnimation(a, t, x, export_video, playback_rate, fignum)
    % FPS for playback and video export
    FPS = 60; % If your computer cannot plot in realtime, lower this.

    % Create objects
    massObj = CubeClass(2*a.r0*[1 1]);
    actuatorObj1 = SpringClass;
    actuatorObj2 = SpringClass;
    
    % colors
    massObj.colors = zeros(8,3);

    % Create a figure handle
    if ~exist('fignum', 'var')
        h.figure = figure;
    else
        h.figure = figure(fignum);
    end
    %This sets figure dimension, dictating video dimensions
    h.figure.Position(3:4) = [1280 720];
    movegui(h.figure)

    % Put the shapes into a plot
    massObj.plot;
    actuatorObj1.plot;
    actuatorObj2.plot;

    % Figure properties
    view(2)
    % title('Simulation')
    xlabel('x Position (m)')
    ylabel('y Position (m)')
    zlabel('z Position (m)')
    % These commands set the aspect ratio of the figure so x scale = y scale
    % "Children(1)" selects the axes that contains the animation objects
    h.figure.Children(1).DataAspectRatioMode = 'manual';
    h.figure.Children(1).DataAspectRatio = [1 1 1];
    set(gcf, 'Color', 'w');
    set(gca, 'Color', 'w');
    % Setup videowriter object
    if export_video
       v = VideoWriter('McKibbenAnimation.mp4', 'MPEG-4');
       v.FrameRate = FPS;
       open(v)
    end

    % Iterate over state data
    tic;
    for t_plt = t(1):playback_rate*1.0/FPS:t(end)

        x_state = interp1(t',x',t_plt);

        % Set axis limits (These will respect the aspect ratio set above)
        h.figure.Children(1).XLim = [0, 1.5];
        h.figure.Children(1).YLim = [-0.2 0.2];
        h.figure.Children(1).ZLim = [-1.0, 1.0];

        % Set positions
        massObj.resetFrame
        massObj.globalMove(SE3([x_state(1) + a.r0, 0, 0]));
        
        actuatorObj1.updateState(SE3, x_state(1));
        actuatorObj1.globalMove(SE3([a.x0(1) 0 0]));
        actuatorObj2.updateState(SE3, x_state(1));
        actuatorObj2.globalMove(SE3([0 0 0 pi 0 0]));

        % Update data
        massObj.updatePlotData
        actuatorObj1.updatePlotData
        actuatorObj2.updatePlotData

        if export_video %Draw as fast as possible for video export
            drawnow
            frame = getframe(h.figure);
            writeVideo(v,frame);
        else % pause until 1/FPS of a second has passed then draw
            while( toc < 1.0/FPS)
                pause(0.002)
            end
            drawnow
            tic;
        end % if exportvideo
    end % t_plt it = ...
end