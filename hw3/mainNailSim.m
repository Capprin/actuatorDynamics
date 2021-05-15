% simulates a series elastic actuator interacting with a nail

clf;

% time
t_max = 2;

% spring parameters
s.m = 1; %toe mass
s.k = 100; %spring const.
s.d = -5; %damping
s.e = 0.85; %restitution
s.l0 = 0.4;

% nail parameters
n.m = 0.01; %TODO: research
n.fs = 1000;
n.fk = 10000;
n.x0 = [0.2 0];

% actuator parameters
a.m = 1; %actuator mass
a.x0 = [1, 0]; %initial state (position, velocity)
a.f = @(t,x) hammerController(t,x,a,s,n);

% run sim
[t_vec, x_vec] = nailSim(a,s,n,t_max);

% plot results
figure(1);
plot(t_vec, [x_vec(1,:); x_vec(3,:); x_vec(5,:)]);
legend('Actuator Position', 'Paddle Position', 'Nail Position');
figure(2);
plot(t_vec, [x_vec(2,:); x_vec(4,:); x_vec(6,:)]);
legend('Actuator Velocity', 'Paddle Velocity', 'Nail Velocity');

% animate results
nailAnimation(t_vec, [x_vec(1,:); x_vec(3,:); x_vec(5,:)], 3, false, 1);