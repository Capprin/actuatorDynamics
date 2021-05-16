% defines constants and does output for a McKibben actuator simulation

% TODO (and questions):
    % figure out extension/contraction
        % when does the actuator do extension? should it, ever?
            % there was some convention disagreement; fixed with a flipped
            % sign in actuatorForce.m
    % reconcile initial length and force behavior:
        % does l0 have to agree with initial eps?
            % no
        % what happens when they don't?
            % elastic properties of actuator rapidly come into play
        % is that correct?
            % probably
    % reasonable pressure control curve (long-period sin wave)
    % validate frictional behavior
        % expect low (static) force, then spike in trans. to kf
        % does friction act in the right direction?
        % why do we see complex values? that isn't right
        % confirm histeresis
            % extension/contraction should have different forces
            % compare with literature values
        % are the constants reasonable?
    % add limits
% next week:
    % more complex dynamics:
        % mass with friction?
        % mass with its own force/position input?
    % closed-loop control
    % pdf sampling for sensing

% actuator parameters
a.l0 = 1;
a.r0 = 0.02;
a.a0 = 2*pi/360*20;
a.x0 = [0; 0];
a.m = 1;
% sheath friction
a.fs = 0.015;
a.fk = 0.105;
a.vf = 0.15; %trans. vel for s-k friction
% behavior
a.s = @(t,x) actuatorSensing(t,x,a); %len/vel as a fn of state
a.c = @(t,x) actuatorControl(t,x,a); %pressure as a fn of state

% load parameters
l.m = 0;
l.x0 = [1.5; 0]; %defines length

% simulation
t_max = 5;
[t_vec, x_vec] = actuatorSim(a,l,t_max);

% plots
figure(1);
% position
subplot(2,1,1);
plot(t_vec, x_vec(:,1));
xlabel('Time (s)');
ylabel('Position (m)');
% EE force
subplot(2,1,2);
eps_vec = (a.l0-x_vec(:,1))/a.l0;
P_vec = actuatorControl(t_vec, x_vec, a);
F_vec = actuatorForce(eps_vec, P_vec, x_vec(:,2), a);
plot(t_vec, F_vec);
xlabel('Time (s)');
ylabel('Force (N)');

% animation
actuatorAnimation(a,t_vec,x_vec(:,1),false,1);
