% computes end-effector force of a McKibben muscle
% from Tondu and Lopez
    % eps: strain (negative in extension)
    % P: pressure
    % vel: velocity
    % a: actuator struct
function [F, F_s, F_sp] = actuatorForce(eps, P, vel, a)
    % static force
    c1 = 3/tan(a.a0)^2;
    c2 = 1/sin(a.a0)^2;
    F_s = pi*a.r0^2.*(P.^(1.8)*1000).*(c1.*(1-eps).^2-c2);
    
    % parallel spring
    F_sp = a.k.*eps.*a.l0 + a.d.*vel;
    
    % passive_dyn spring
    k_pass = 5;
    c_pass = 30;
    F_pass = k_pass.*eps.*a.l0 + c_pass.*vel;
    
    
    % dynamic force (negative for contraction)
    F = -(F_s - F_sp - F_pass);
    % address complex values
    if ~isreal(F)
        warning('Model in extension causes residual imaginary components')
        [~, id] = lastwarn;
        warning('off',id);
        % fix force
        F = real(F);
    end
end