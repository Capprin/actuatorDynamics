function [t_vec, x_vec] = actuatorSim(a,s,m, t_max)
    % ode45 setup
    x0 = [s.x0-a.x0, a.x0, m.x0]';
    event_fun = @(t,x) contactEvent(t,x,a,s,m);
    opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', event_fun);
    dyn_fun = @(t,x) uncoupled_dynamics(t,x,a,s,m); %assume start uncoupled
    t = 0;
    t_vec = [];
    x_vec = [];
    while t < t_max
        % run ode solver until make/break contact
        sol = ode45(dyn_fun, [t t_max], x0, opts);
        % append soln
        t_vec = [t_vec sol.x(2:end)];
        sol.y = sol.y(:,2:end); %get rid of initial point
        % setup initial conditions for next run
        t = sol.x(end);
        % interpret event
        if length(x0) == 4
            % was coupled, just decoupled
            disp(['decoupled at t=',num2str(t)]);
            dyn_fun = @(t,x) uncoupled_dynamics(t,x,a,s,m);
            x0 = [sol.y(:,end);...
                  sol.y(1, end)+sol.y(3, end);...
                  s.e*sol.y(2, end)+1/10]; %includes restitution
            x_vec = [x_vec [sol.y; sol.y(1:2,:)+sol.y(3:4,:)]];
        else
            % was decoupled, just coupled
            disp(['coupled at t=',num2str(t)]);
            dyn_fun = @(t,x) coupled_dynamics(t,x,a,s,m);
            x0 = [sol.y(1,end);...
                  (sol.y(2,end)*s.m + sol.y(6,end)*m.m)/(s.m+m.m);... %inelastic
                  sol.y(3:4,end)];
            x_vec = [x_vec sol.y];
        end
    end
end

function dx = coupled_dynamics(t,x,a,s,m)
    v_s = x(2);
    a_s = -(s.k*(s.x0(1)-a.x0(1)-x(1)) + s.d*x(2))/(s.m+m.m) - a.f(t,x)/a.m - 9.8;
    v_a = x(4);
    a_a = a.f(t,x)/a.m-9.8-a_s*m.m/a.m;
    dx = [v_s a_s v_a a_a]';
end

function dx = uncoupled_dynamics(t,x,a,s,m)
    v_s = x(2);
    a_s = -(s.k*(s.x0(1)-a.x0(1)-x(1)) + s.d*x(2))/s.m - a.f(t,x)/a.m - 9.8;
    v_a = x(4);
    a_a = a.f(t,x)/a.m-9.8-a_s*m.m/a.m;
    v_m = x(6);
    a_m = -9.8;
    dx = [v_s a_s v_a a_a v_m a_m]';
end

function [event_val,is_terminal,direction] = contactEvent(t,x,a,s,m)
    % event setup
    is_terminal = 1; %always terminate at a zero
    direction = -1; %breaks regardless of direction
    if length(x) == 4
        % currently coupled; looking for accel. difference
        dx = coupled_dynamics(t,x,a,s,m);
        event_val = dx(2)*(m.m+s.m)/s.m + 9.8;
    else
        % currently uncoupled; looking for pos. difference
        event_val = (x(5) - x(1) - x(3));
    end
end

% problems were:
    % incorrect motion eqns
    % multiplicative collision velocities
    % lack of reaction forces
    % STICKY PLATFORM
    % incorrect animation positions
    % use of spring displacements instead of world positions