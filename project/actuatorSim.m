% does dynamics simulation for a McKibben actuator and load in 1D
% hybrid sim allows for contact events
function [t_vec, x_vec] = actuatorSim(a, l, t_max)
    % set up ode45
    x0 = a.x0 + l.x0;
    [t_vec, x_vec] = ode45(@(t,x) dynamics(t,x,a,l), [0 t_max], x0);
end

% state: a_pos, a_vel, l_pos, l_vel
function dx = dynamics(t,x,a,l)
    % velocity
    vel = x(2);
    % compute force
    eps = (a.l0-x(1))/a.l0;
    P = a.c(t,x);
    F = actuatorForce(eps,P,vel,a) + l.f(t,x);
    % accelerate both actuator and load at once
    accel = F/(a.m+l.m);
    dx = [vel; accel];
end