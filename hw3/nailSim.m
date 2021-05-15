%TODO: add exit condition when nail is "in"

% simulates series elastic hammer + nail
function [t_vec, x_vec] = nailSim(a,s,n,t_max)
    % define IC, dynamics, events
    is_ballistic = true; %assume initially ballistic
    x0 = [a.x0, a.x0-s.l0, n.x0]';
    event_fun = @(t,x) contactEvent(t,x,a,s,n,is_ballistic);
    opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', event_fun);
    dyn_fun = @(t,x) dynamics(t,x,a,s,n,is_ballistic);
    % set up loop vars
    t = 0;
    t_vec = [];
    x_vec = [];
    while t < t_max
        % run ODE solver to simulate motion
        sol = ode45(dyn_fun, [t t_max], x0, opts);
        % append results, ignoring intermediate IC
        t_vec = [t_vec sol.x(2:end)];
        x_vec = [x_vec sol.y(:,2:end)];
        % set intermediate IC for next iteration
        t = sol.x(end);
        x0 = x_vec(:,end); %TODO: might need a "kick"
        % change models
        % check if nail is bottomed out
        if x0(5) <= 0 && ~is_ballistic
            % reset velocities
            x0(4) = 0;
            x0(6) = 0;
        elseif is_ballistic
            x0(4) = 0; % reset velocity of paddle
            is_ballistic = false;
        elseif ~is_ballistic
            x0(6) = 0;
            x0(4) = x0(4);
            is_ballistic = true;
        end
        str = {'false', 'true'};
        disp(['is_ballistic=' str{is_ballistic+1} ' at t=' num2str(t)]);
        event_fun = @(t,x) contactEvent(t,x,a,s,n,is_ballistic);
        opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', event_fun);
        dyn_fun = @(t,x) dynamics(t,x,a,s,n,is_ballistic);
    end
end

% dynamics fn for system
% order: actuator, paddle, nail
function dx = dynamics(t,x,a,s,n,is_ballistic)
    % forces
    f_s = s.k*(s.l0 - (x(1)-x(3))) + s.d*(x(2)-x(3)); %spring
    f_p = -f_s; %paddle
    if x(3) > 0
        f_n = nail_response(n, f_p - 9.8*n.m);
    else
        x(4) = 0;
        f_n = -f_p; %normal response when on ground
    end
    % actuator accel.
    v_a = x(2);
    a_a = (f_s + a.f(t,x))/a.m - 9.8;
    % paddle accel.
    v_p = x(4);
    a_p = (f_p + ~is_ballistic*f_n)/a.m - 9.8; %could be wrong
    % nail accel.
    if is_ballistic || a_p > 0
        v_n = 0;
        a_n = 0;
    elseif ~is_ballistic && x(5) <= 0
        v_n = 0;
        a_n = 0;
    else
        v_n = x(6);
        a_n = a_p;
    end
    % constr. state incl. nail
    dx = [v_a, a_a, v_p, a_p, v_n, a_n]';
end

% events to switch models
function [event_val, is_terminal, direction] = contactEvent(t,x,a,s,n,is_ballistic)
    % setup
    is_terminal = 1; %always terminate at a zero
    direction = 0; %ignore direction
    if is_ballistic
        event_val = x(3)-x(5); %use position difference
    else
        dx = dynamics(t,x,a,s,n,is_ballistic);
        event_val = (dx(4)-9.8)*x(5); %either paddle accel. 0 or nail bottomed
    end
end

function f = nail_response(n,f)
    if f > 0
        f = 0; %no force returned for pos. input
    elseif abs(f) <= n.fs
        f = -f;
        return; %reflect f (done for clarity)
    else
        f = n.fk; %can only do kinetic friction
    end
end