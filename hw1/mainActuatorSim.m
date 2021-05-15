% simulates an actuator in series with a spring and damper.
% interacts occasionally with a mass, which otherwise behaves ballistically

clf;

% time
t_max = 3.5;

% actuator parameters
a.m = 1; %actuator mass
a.f = @(t,x) -100000*(x(3)); %input force (time, state)
a.x0 = [0, 0]; %initial state (position, velocity)

% spring parameters
s.m = 0.1; %toe mass
s.k = -50; %spring const.
s.d = 0; %damping
s.e = 0.85; %restitution
s.x0 = [1, 0];

% free mass parameters
m.m = 1;
m.x0 = [2, 0];

% run simulation
[t_vec, x_vec] = actuatorSim(a,s,m,t_max);

% plot results
figure(1);
%subplot(2,1,1);
plot(t_vec, [x_vec(1,:)+x_vec(3,:); x_vec(3,:); x_vec(5,:)]);
legend('Spring', 'Actuator', 'Mass');
xlabel('Time (s)');
ylabel('Position (m)');
%subplot(2,1,2);
%plot(t_vec, [x_vec(2,:); x_vec(4,:); x_vec(6,:)]);
%legend('Spring', 'Actuator', 'Mass');
%xlabel('Time (s)');
%ylabel('Velocity (m/s)');


% animate results
actuatorAnimation(t_vec, x_vec, 2, false, 1);
