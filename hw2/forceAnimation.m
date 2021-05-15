% animates behavior from forceSim.m
% much of this logic is borrowed from paddleAnimation.m
function forceAnimation(t_vec, x_vec, fignum, exportVideo, playbackRate)
    FPS = 60;
    
    % widths
    a_w = 0.3;
    l_w = 1;

    % create objects
    actuator = CubeClass([a_w, 1]);
    damper = CubeClass;
    spring = SpringClass;
    load = CubeClass([l_w, 1]);
    
    % set colors (this doesn't work lmao)
    set(actuator.plotHandle, 'Color',[143, 0, 24]);
    set(load.plotHandle, 'Color', 'black');
    set(spring.plotHandle, 'Color', [0, 128, 0]);
    set(damper.plotHandle, 'Color', [0, 128, 0]);
    
    % create a figure handle
    h.figure = figure(fignum);
    % set figure dimension, dictating video dimensions
    h.figure.Position(3:4) = [1280 720];
    movegui(h.figure)
    
    % put objects in plot
    actuator.plot;
    damper.plot;
    spring.plot;
    load.plot;
    
    % set up figure
    view(2)
    title('Simulation')
    xlabel('x Position (m)')
    ylabel('y Position (m)')
    zlabel('z Position (m)')
    % set the aspect ratio of the figure so x scale = y scale
    % "Children(1)" selects the axes that contains the animation objects
    h.figure.Children(1).DataAspectRatioMode = 'manual';
    h.figure.Children(1).DataAspectRatio = [1 1 1];
    
    % set up videowriter object
    if exportVideo
       v = VideoWriter('actuatorAnimation.mp4', 'MPEG-4');
       v.FrameRate = FPS;
       open(v)
    end
    
    % iterate over state data
    tic;
    for t_plt = t_vec(1):playbackRate*1.0/FPS:t_vec(end)
        
        x_state = interp1(t_vec,x_vec,t_plt);

        % set axis limits (These will respect the aspect ratio set above)
        h.figure.Children(1).XLim = [-2, 4];
        h.figure.Children(1).YLim = [0 1];
        h.figure.Children(1).ZLim = [-1.0, 1.0];

        % set positions
        actuator.resetFrame;
        actuator.globalMove(SE3([x_state(1)-a_w/2, 0.5, 0]));
        load.resetFrame;
        load.globalMove(SE3([x_state(2)+l_w/2, 0.5, 0]));
        % draw spring
        spring.resetFrame;
        spring.updateState(SE3([x_state(1),0.75,0,0,0,0]), x_state(2)-x_state(1));
        % hack damper
        damper_handle = damper.plotHandle;
        damper = CubeClass([x_state(2)-x_state(1), 0.075]);
        damper.plotHandle = damper_handle;
        damper.globalMove(SE3([x_state(1)+(x_state(2)-x_state(1))/2, 0.25, 0]));

        % Update data
        actuator.updatePlotData
        load.updatePlotData
        spring.updatePlotData
        damper.updatePlotData

        if exportVideo %Draw as fast as possible for video export
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