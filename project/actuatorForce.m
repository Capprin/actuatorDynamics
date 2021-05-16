% computes end-effector force of a McKibben muscle
% from Tondu and Lopez
    % eps: strain (negative in extension)
    % P: pressure
    % vel: velocity
    % a: actuator struct
function F = actuatorForce(eps, P, vel, a)
    if eps < 0
        warning('Model in extension causes residual imaginary components')
        [~, id] = lastwarn;
        warning('off',id);
    end

    % static force
    c1 = 3/tan(a.a0)^2;
    c2 = 1/sin(a.a0)^2;
    F_s = pi*a.r0^2.*P.*(c1.*(1-eps).^2-c2);
    
    % coeff. of friction (using approx. contact area)
    S = 2.*pi.*a.r0.*a.l0.*sin(a.a0)./((1-eps).*sqrt(1-cos(a.a0).^2.*(1-eps).^2));
    mu = a.fk + (a.fs - a.fk)*exp(vel/a.vf);
    
    % dynamic force (negative for contraction)
    F = -(F_s - mu.*S.*P.*sign(vel));
    if eps < 0
        F = real(F);
    end
end