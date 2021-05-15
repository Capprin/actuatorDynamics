% computes end-effector force of a McKibben muscle
% from Tondu and Lopez
function F = actuatorForce(eps, P, v, a)
    % static force
    % TODO: gets impossibly huge with pressure. check units
    c1 = 3/tan(a.a0)^2;
    c2 = 1/sin(a.a0)^2;
    F_s = pi*a.r0^2*P.*(c1*(1-eps).^2-c2);
    
    % coeff. of friction (using approx. contact area)
    % TODO: behaves weirdly for stretched actuator
    S = 2*pi*a.r0*a.l0*sin(a.a0)/((1-eps).*sqrt(1-cos(a.a0)^2*(1-eps).^2));
    mu = a.fk + (a.fs - a.fk)*exp(v/a.vf);
    
    % dynamic force
    F = F_s - mu*S*P*sign(v);
end