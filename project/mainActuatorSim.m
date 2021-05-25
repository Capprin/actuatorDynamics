% defines constants and does output for a McKibben actuator simulation
    
% TODO:
    % hack spring animation more:
        % make radius accurate
        % expand radially with contraction
        % reflect sheath angle alpha
    % add limits
        % length
        % pressure:
            % absolute
            % rate of change
            % control curve
    % closed-loop control
    % pdf sampling for sensing

% actuator parameters
a.l0 = 1;
a.r0 = 0.02;
a.a0 = 2*pi/360*20;
a.x0 = [0; 0];
a.m = 1;
% parallel spring parameters
a.k = 200;
a.d = -30;
% sheath friction
a.do_f = false;
a.fs = 0.015;
a.fk = 0.105;
a.vf = 0.15; %trans. vel for s-k friction
% behavior
a.s = @(t,x) actuatorSensing(t,x,a); %len/vel as a fn of state
a.c = @(t,x) actuatorControl(t,x,a); %pressure as a fn of state
% desired behavior
per = 1;
freq = 1/per;
a.x_des = @(t) (7/8)+(cos(t.*pi./freq))./8;
a.dx_des = @(t) -sin(t.*pi./freq)./8.*pi./freq;

% load parameters
l.m = 1;
l.x0 = [1; 0]; %defines length
l.f = @(t,x) 0;

% simulation
t_max = 10;
[t_vec, x_vec] = actuatorSim(a,l,t_max);

% plots
figure(1);
set(gcf, 'Position',  [100, 100, 1000, 800])
% position
subplot(3,1,1);
plot(t_vec, x_vec(:,1));
xlabel('Time (s)');
ylabel('Position (m)');
title('McKibben End Position');
% legend('End effector position', 'Desired position');
% EE force
subplot(3,1,2);
eps_vec = (a.l0-x_vec(:,1))/a.l0;
% pressure
P = arrayfun(@(t,x1, x2) actuatorControl(t,[x1 x2],a), t_vec, x_vec(:,1), x_vec(:,2));
P = P*14.504;
P_vec = actuatorControl(t_vec, x_vec, a);
[F_vec, F_s, F_f, F_sp] = actuatorForce(eps_vec, P_vec, x_vec(:,2), a);
plot(t_vec, F_vec);
xlabel('Time (s)');
ylabel('Force (N)');
title('McKibben Aggregate End Force');
subplot(3,1,3);
plot(t_vec, [-F_s, F_f, F_sp]);
xlabel('Time (s)');
ylabel('Force (N)');
title('McKibben Force Components');
legend('Static Force', 'Frictional Force', 'Spring Force');
% pressure plot
figure(3);
set(gcf, 'Position',  [100, 100, 1000, 800])
plot(t_vec,P);
title('McKibben Internal Presssure');
xlabel('Time (s)');
ylabel('Pressure (psi)');

% animation
figure(2);
clf;
actuatorAnimation(a,t_vec,x_vec(:,1),false,1,2);
