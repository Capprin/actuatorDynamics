clf;

% time
t_max = 5;

% spring parameters
s.l0 = 2;
s.k = -40;
s.d = -1;

% load parameters
l.m = 1;

% actuator parameters
a.m = 1;
a.x0 = [0; 0];
a.f_max = 50;

% simulate for many frequencies
freqs = linspace(0,5);
ratios = zeros(size(freqs));
for i = 1:length(freqs)
    freq = freqs(i);
    %l.x = @(t) sin(t*freq*2*pi) + 2; %oscillation
    %l.v = @(t) freq*2*pi*cos(t*freq*2*pi);
    %l.a = @(t) -freq^2*4*pi*pi*sin(t*freq*2*pi);
    l.x = @(t) ones(size(t))*2;
    l.v = @(t) 0;
    l.a = @(t) 0;
    %a.f_des = @(t) ones(size(t));
    a.f_des = @(t) 2*sin(2*pi*freq*t);
    a.f = @(t,x) forceController(t,x,a,s,l);
    [t, ~, f] = forceSim(s,a,l,t_max);
    % compute ratio
    ratio = abs(a.f_des(t)./f);
    ratios(i) = mean(ratio(~isnan(ratio)));
end

% plot results
plot(freqs, ratios);
xlabel('Frequency (Hz)');
ylabel('Force Ratio (des:act)');
title('Frequency Performance, Continuous Force');