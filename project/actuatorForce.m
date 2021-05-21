% computes end-effector force of a McKibben muscle
% from Tondu and Lopez
    % eps: strain (negative in extension)
    % P: pressure
    % vel: velocity
    % a: actuator struct
function [F, F_s, F_f, F_sp] = actuatorForce(eps, P, vel, a)
    % sanitize
    if eps < 0
        warning('Model in extension causes residual imaginary components')
        [~, id] = lastwarn;
        warning('off',id);
    end

    % static force
    c1 = 3/tan(a.a0)^2;
    c2 = 1/sin(a.a0)^2;
    F_s = pi*a.r0^2.*P.*(c1.*(1-eps).^2-c2);
    
    % frictional force
    if a.do_f
        S = 2.*pi.*a.r0.*a.l0.*sin(a.a0)./((1-eps).*sqrt(1-cos(a.a0).^2.*(1-eps).^2));
        mu = a.fk + (a.fs - a.fk)*exp(vel/a.vf);
        F_f = mu.*S.*P.*sign(vel);
    else
        F_f = zeros(size(F_s));
    end
    
    % parallel spring
    F_sp = a.k.*eps.*a.l0 + a.d.*vel;
    
    % dynamic force (negative for contraction)
    F = -(F_s - F_f - F_sp);
    if eps < 0
        F = real(F);
    end
end