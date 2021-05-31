% defines constants and does output for a McKibben actuator simulation

% actuator parameters
a.l0 = 1;
a.r0 = 0.02;
a.a0 = 2*pi/360*20;
a.x0 = [0; 0; 0];
% parallel spring parameters
a.k = 200;
a.d = -30;
% limits
a.p_max = 60;
a.p_min = 0;
a.dp_max = 110;
% desired behavior
freq = 3/2; %cycles/second
a.x_des = @(t) (7/8)+(cos(2*t.*pi.*freq))./8;
a.dx_des = @(t) -sin(2*t.*pi.*freq)./8*2.*pi.*freq;
% behavior
a.s = @(t,x) actuatorSensing(t,x,a); %len/vel as a fn of state
a.c = @(t,x) actuatorControl(t,x,a); %pressure as a fn of state

% load parameters
l.m = 1;
l.x0 = [1; 0]; %defines length
l.f = @(t,x) 0;

% simulation
t_max = 5;
[t_vec, x_vec] = actuatorSim(a,l,t_max);

% plots
figure(1);
set(gcf, 'Position',  [100, 100, 1000, 800])
% position
subplot(4,1,1);
plot(t_vec, [x_vec(:,1) a.x_des(t_vec)]);
xlabel('Time (s)');
ylabel('Position (m)');
title('McKibben End Position');
legend('End effector position', 'Desired position');
% pressure
subplot(4,1,2);
P_vec = x_vec(:,3);
P_des = arrayfun(@(t,x,v) a.c(t, [x v]), t_vec, x_vec(:,1), x_vec(:,2));
cla;
hold on;
plot(t_vec, 14.504*[P_vec P_des]); %adjusted bar -> psi
plot(t_vec, zeros(size(t_vec)), '--');
hold off;
title('McKibben Internal Presssure');
xlabel('Time (s)');
ylabel('Pressure (psi)');
legend('Internal Pressure', 'Desired Pressure');
% EE force
subplot(4,1,3);
eps_vec = (a.l0-x_vec(:,1))/a.l0;
[F_vec, F_s, F_sp] = actuatorForce(eps_vec, P_vec, x_vec(:,2), a);
plot(t_vec, F_vec);
xlabel('Time (s)');
ylabel('Force (N)');
title('McKibben Aggregate End Force');
subplot(4,1,4);
plot(t_vec, [-F_s, F_sp]);
xlabel('Time (s)');
ylabel('Force (N)');
title('McKibben Force Components');
legend('Static Force', 'Spring Force');

figure(2);
dP_vec = arrayfun(@(p_des, p_curr) pressureControl(a, p_des, p_curr), P_des, x_vec(:,3));
plot(t_vec, 14.504*dP_vec);
title('Differential Pressure');
xlabel('Time (s)');
ylabel('P/s (psi)');

% animation
figure(3);
clf;
actuatorAnimation(a,t_vec,x_vec(:,1),false,1,3);
