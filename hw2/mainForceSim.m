% simulates an actuator in series with a spring and damper.
% optimizes actuator force to maintain force on a moving object

clf;

% time
t_max = 5;

% spring parameters
s.l0 = 2;
s.k = -85;
s.d = -1;

% load parameters
l.x = @(t) sin(t*2*pi) + 2; %oscillation
l.v = @(t) 2*pi*cos(t*2*pi);
l.a = @(t) -4*pi*pi*sin(t*2*pi);
%l.x = @(t) ones(size(t))*2;
%l.v = @(t) 0;
%l.a = @(t) 0;
l.m = 1;

% actuator parameters
a.m = 1;
a.x0 = [0; 0];
a.f_des = @(t) ones(size(t));
%a.f_des = @(t) 2*sin(2*pi*t);
a.f_max = 50;
a.f = @(t,x) forceController(t,x,a,s,l);

[t_vec, x_vec, f_vec] = forceSim(s,a,l,t_max);
f_con = zeros(size(f_vec));
for i = 1:length(t_vec)
    f_con(i) = a.f(t_vec(i), x_vec(i,:));
end

% plot results
figure(1);
subplot(2,1,1);
plot(t_vec, [x_vec(:,1), l.x(t_vec)]); %positions
xlabel('Time (s)');
ylabel('Position(m)');
legend('Actuator', 'Wall');
subplot(2,1,2);
plot(t_vec, [f_con, f_vec, a.f_des(t_vec)]); %force
xlabel('Time (s)');
ylabel('Load (N)');
legend('Actuator Force', 'Wall Load', 'Target Wall Load');

% animate results
forceAnimation(t_vec, [x_vec(:,1), l.x(t_vec)], 2, false, 1);