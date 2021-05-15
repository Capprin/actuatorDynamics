% simulates series spring-mass actuator
function [t_vec, x_vec, f_vec] = forceSim(s, a, l, t_max)
    x0 = [a.x0];
    s.f = @(t,x) s.k*((l.x(t)-x(1)) - s.l0) + s.d*(l.v(t)-x(2));
    dyn_fun = @(t,x) [x(2);... %actuator velocity
                      (a.f(t,x) - s.f(t,x))/a.m]; %actuator accel.
    [t_vec, x_vec] = ode45(dyn_fun, [0 t_max], x0);
    % save forces (this is unclean, but can't find a better way
    f_vec = zeros(length(t_vec),1);
    for i = 1:length(f_vec)
        f_vec(i) = s.f(t_vec(i), x_vec(i,:)');
    end
end