% does dynamics simulation for a McKibben actuator and load in 1D
% hybrid sim allows for contact events
function [t_vec, x_vec] = actuatorSim(a, l, t_max)
    % set up ode45
    x0 = a.x0;
    % set time parameters
    t_start = 0;
    t_end = t_max;
    dt = 0.005;
    % bind dynamics functions
    dynamics_attached = @(t,x) dynamicsAttached(t,x,a,l);
    dynamics_free = @(t,x) dynamicsFree(t,x,a,l);
    dynamics_nail = @(t,x) dynamicsNail(t,x,a,l);
    % bind event function
    event_fun = @(t,x) contactEvent(t,x,a,l,dynamics_attached);
    % sim tolerances
    options = odeset(...
    'RelTol', 1e-10, ...
    'AbsTol', 1e-10, ...
    'Events',event_fun);
    % data structures
    t_vec = t_start:dt:t_end;
    x_vec = zeros(length(x0), length(t_vec));
    sol_set = {};
    
    % simulate dynamics
    while t_start < t_end
        if isempty(sol_set)
            sol = ode45(dynamics_attached, [t_start,t_end], x0, options);
        elseif sol.ie(end) == 1
            sol = ode45(dynamics_free, [t_start,t_end], x0, options);
        elseif sol.ie(end) == 2
            sol = ode45(dynamics_nail, [t_start,t_end], x0, options);
        end
        % concatenate sol_set
        sol_set = [sol_set, {sol}];
        % set up next call
        t_start = sol.x(end);
        % apply hybrid map, calc post contact velocities
        if ~isempty(sol.ie) && sol.ie(end) == 1 % first event load-paddle contact
            x0 = mckibbenContactMap(sol.xe(end),sol.ye(:,end),a,l);
            disp(t_start)
            disp('Detach!')
        end
        if ~isempty(sol.ie) && sol.ie(end) == 2 % first event load-paddle contact
            x0 = nailContactMap(sol.xe(end),sol.ye(:,end),a,l);
            disp('Nailed!')
        end
    end
    
    % Loop to sample the solution structures and built X_vec
    for idx = 1:length(sol_set)
        % This sets up a logical vector so we can perform logical indexing
        t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
        if ~any(t_sample_mask)
            continue
        end
        % Evaluate the idx solution structure only at the applicable times
        X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
        % Assign the result to the correct indicies of the return state array
        x_vec(:,t_sample_mask) = X_eval;
    end
    
    % use transpose to fix plots (def a better fix for this)
    x_vec = x_vec';
end

% state: a_pos, l_pos, n_pos, a_vel, l_vel, n_vel

% dynamics with the load attached to the McKibben
function dx = dynamicsAttached(t,x,a,l)
    % velocity
    vel = x(4);
    % compute force
    eps = (a.l0-x(1))/a.l0;
    F = actuatorForce(eps,x(7),vel,a) + l.f(t,x);
    % update pressure with control
    p_des = a.c(t,x);
    dP = pressureControl(a, p_des, x(7));
    % accelerate both actuator and load at once
    accel = F/l.m;
    dx = [vel; vel; 0; accel; accel; 0; dP];
end

% dynamics when McKibben has no load, and the load is moving towards the
% nail
function dx = dynamicsFree(t,x,a,l)
    % velocity
    vel_m = x(4);
    vel_l = x(5);
    % compute force
    eps = (a.l0-x(1))/a.l0;
    P = 0;
    dP = pressureControl(a, P, x(7));
    F = actuatorForce(eps,x(7),vel_m,a) + l.f(t,x);
    % accelerate both actuator and load at once
    accel = F/l.m;
    dx = [vel_m; vel_l; 0; accel; 0; 0; dP];
end

% dynamics with load contacted with nail. Inelastic colision, constant
% friction force upon the nail/load until velocities are 0
function dx = dynamicsNail(t,x,a,l)
    % velocity
    vel_m = x(4);
    if x(5) < 0
        vel_l = 0;
    else
        vel_l = x(5);
    end
    % compute force
    eps = (a.l0-x(1))/a.l0;
    P = 0;
    dP = pressureControl(a, P, x(7));
    F = actuatorForce(eps,x(7),vel_m,a) + l.f(t,x);
    F_friction = -1;
    % accelerate both actuator and load at once
    accel_m = F/l.m;
    accel_l = F_friction/l.m;
    
    
    dx = [vel_m; vel_l; vel_l; accel_m; accel_l; accel_l; dP];
end

% contact function to determine the two contact events
function [eventVal,isterminal,direction] = contactEvent(t,x,a,l,dynamics_attached)
    % obtain current vel and acc values
    dx = dynamics_attached(t,x);
    
    % to avoid contact event at first retraction, make sure there's a
    % certain time before and after to fit within a specific window
    if 1.5 < t && t < 2.0
        mckibben_acc = dx(4);
    else
        mckibben_acc = 0;
    end
    
    % distance between the load and nail, to determine nail contact event
    dist_load = x(2)-x(3);
    
    eventVal = [mckibben_acc, dist_load];
    isterminal = [1, 1];
    direction = [-1, 1];
end % contactEvent

% detaching from the McKibben doesn't change the vel and acc
function X_post = mckibbenContactMap(t,x_pre,a,l)
    X_post = x_pre;
end

% inelastic contact with the nail results in identical vel and acc post
% impact
function X_post = nailContactMap(t,x_pre,a,l)
    X_post = x_pre;
    X_post(3) = x_pre(2);
    X_post(6) = x_pre(5);
end
