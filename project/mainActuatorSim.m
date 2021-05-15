% defines constants and does output for a McKibben actuator simulation

% actuator parameters
a.l0 = 1;
a.r0 = 0.02;
a.a0 = 2*pi/360*20;
a.x0 = [0; 0];
a.m = 1;
% TODO: add limits
% sheath friction
a.fs = 0.015;
a.fk = 0.105;
a.vf = 0.15; %trans. vel for s-k friction
% behavior
a.s = @(t,x) actuatorSensing(t,x,a); %len/vel as a fn of state
a.c = @(t,x) actuatorControl(t,x,a); %pressure as a fn of state

% load parameters
l.m = 0;
l.x0 = [0.5; 0]; %defines length

% simulation
t_max = 5;
[t_vec, x_vec] = actuatorSim(a,l,t_max);

% plots and animation
figure(1);
% position
subplot(2,1,1);
plot(t_vec, x_vec(:,1));
xlabel('Time (s)');
ylabel('Position (m)');
% EE force
subplot(2,1,2);
eps_vec = (a.l0 - x_vec(:,1))/a.l0;
P_vec = actuatorControl(t_vec, x_vec, a);
F_vec = actuatorForce(eps_vec, P_vec, x_vec(:,2), a);
plot(t_vec, F_vec);
xlabel('Time (s)');
ylabel('Force (N)');
