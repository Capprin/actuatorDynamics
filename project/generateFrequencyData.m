% standalone script to generate frequency plots
% does a surface error plot w.r.t. amplitude, frequency\

% stuff to change
offset = 7/8;
amplitudes = 1/32:1/32:1/4;
frequencies = 1:10;
t_max = 15;

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

% load parameters
l.m = 1;
l.x0 = [1; 0]; %defines length
l.f = @(t,x) 0;

% generate data
errors = zeros(length(amplitudes), length(frequencies));
for am = 1:length(amplitudes)
    for fr = 1:length(frequencies)
        % setup
        amp = amplitudes(am);
        freq = frequencies(fr);
        % parameter-specific functions
        a.x_des = @(t) offset + amp*cos(2*pi*t*freq);
        a.dx_des = @(t) -amp*2*pi*freq*sin(2*pi*t*freq);
        a.c = @(t,x) actuatorControl(t,x,a);
        % simulation, error
        [t_vec, x_vec] = actuatorSim(a,l,t_max);
        error_vec = abs((x_vec(1) - a.x_des(t_vec)')./a.x_des(t_vec)');
        errors(am,fr) = mean(error_vec);
    end
end
errors = errors'; %transpose for surface plot

% produce ribbon plot
[X,Y] = meshgrid(amplitudes, frequencies);
ribbon(frequencies, 100*errors);
[num, dem] = rat(amplitudes);
labels = arrayfun(@(n,d) [num2str(n) '/' num2str(d)], num, dem, 'UniformOutput', false);
xticks(linspace(0,10,length(amplitudes)));
xticklabels(labels);
xlabel('Amplitude (m)');
ylabel('Frequency (Hz)');
zlabel('%Error');
title('McKibben Periodic Task Performance');