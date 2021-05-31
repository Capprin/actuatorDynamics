% does dynamics simulation for a McKibben actuator and load in 1D
% hybrid sim allows for contact events
function [t_vec, x_vec] = actuatorSim(a, l, t_max)
    % set up ode45
    x0 = a.x0 + [l.x0; 0];
    [t_vec, x_vec] = ode45(@(t,x) dynamics(t,x,a,l), [0 t_max], x0);
end

% simulates McKibben dynamics under control
% state: [position, velocity, pressure]
function dx = dynamics(t,x,a,l)
    % velocity
    vel = x(2);
    % compute force using pressure
    eps = (a.l0-x(1))/a.l0;
    F = actuatorForce(eps,x(3),vel,a) + l.f(t,x);
    % update pressure with control
    p_des = a.c(t,x);
    dP = pressureControl(a, p_des, x(3));
    % accelerate both actuator and load at once
    accel = F/l.m;
    dx = [vel; accel; dP];
end

