% animates behavior from actuatorSim.m
% much of this logic is borrowed from paddleAnimation.m
function actuatorAnimation(t_vec, x_vec, fignum, exportVideo, playbackRate)
    FPS = 60;
    
    % heights
    a_h = 0.3;
    p_h = 0.1;
    m_h = 0.2;

    % create objects
    actuator = CubeClass([0.5, a_h]);
    paddle = CubeClass([0.5, p_h]);
    damper = CubeClass;
    spring = SpringClass;
    mass = CubeClass([0.4, m_h]);
    
    % create a figure handle
    h.figure = figure(fignum);
    % set figure dimension, dictating video dimensions
    h.figure.Position(3:4) = [720 1280];
    movegui(h.figure)
    
    % put objects in plot
    actuator.plot;
    paddle.plot;
    damper.plot;
    spring.plot;
    mass.plot;
    
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
        
        x_state = interp1(t_vec',x_vec',t_plt);

        % set axis limits (These will respect the aspect ratio set above)
        h.figure.Children(1).XLim = [-0.6, 0.6];
        h.figure.Children(1).YLim = [-0.5, 3];
        h.figure.Children(1).ZLim = [-1.0, 1.0];

        % set positions
        actuator.resetFrame;
        actuator.globalMove(SE3([0, x_state(3)-a_h/2, 0]));
        mass.resetFrame;
        mass.globalMove(SE3([0, x_state(5)+m_h/2, 0]));
        paddle.resetFrame;
        paddle.globalMove(SE3([0, x_state(1)-p_h/2+x_state(3), 0]));
        % draw spring
        spring.resetFrame;
        spring.updateState(SE3([-0.12,x_state(3),0,0,0,pi/2]), x_state(1)-p_h); %beneath paddle
        % hack damper
        damper_handle = damper.plotHandle;
        damper = CubeClass([0.075, x_state(1) - p_h]);
        damper.plotHandle = damper_handle;
        damper.globalMove(SE3([0.12, (x_state(1) - p_h)/2+x_state(3), 0]));

        % Update data
        actuator.updatePlotData
        mass.updatePlotData
        paddle.updatePlotData
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