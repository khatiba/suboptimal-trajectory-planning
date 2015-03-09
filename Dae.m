%------------------------------------
% BEGIN: function Dae.m
%------------------------------------

function ode = Dae(~,y,P)

    global param
    
    ode = zeros(size(y));

    r        = y(1);
    phi      = y(3);
    V        = y(4);
    gamma    = y(5);
    psi      = y(6);
    S        = y(7);
    sigma    = y(8);
    sigmad   = y(9);

    E = (1/2).*(V.^2) - param.mu.*(r.^-1);
    En = (E - param.E0) / (param.Ef - param.E0);
    
    % Coriolis Terms
    Cpsi = 2*param.omega*(tan(gamma)*sin(psi)*cos(phi)-sin(phi));
    Cgamma = 2*param.omega*cos(psi)*cos(phi);

    % Altitude
    h = r-param.r_eq;

    % Density & Speed of sound
    [rho Vs] = Atmosph(h);

    % Mach Number
    M = V./Vs;

    % Aerodynamics coefficients and drag acceleration
    [CD CL] = AeroCoeff(M);
    D = 0.5*rho.*CD.*V.^2*(param.Area/param.mass);
    L = 0.5*rho.*CL.*V.^2*(param.Area/param.mass);

    % Gravitational Acceleration
    rsq = r.^-2;
    g = param.mu*rsq;
    sigmadr = 0;
    
    sigmar  = param.sigmarMin;
    
    if (En > P(1))
        sigmar = param.sigmarMax;
    end
    if (En > P(2))
        sigmar = -param.sigmarMax;
    end
    if (En > P(3))
        sigmar = param.sigmarMin;
    end
    if (En >= 0.96)
        sigmar = param.sigmarMin;
    end

    % Control and Control Rate Constraints
    sigmar = Saturate(sigmar, -param.sigmarMax, param.sigmarMax);

    % Equations of Motion
    dr        =  V.*sin(gamma);
    dtheta    =  V.*(r.^-1).*cos(gamma).*cos(psi).*(cos(phi).^-1);
    dphi      =  V.*(r.^-1).*cos(gamma).*sin(psi);
    dV        = -D -g.*sin(gamma);
    dgamma    =  (V.^-1).*(L.*cos(sigma)-(g-(r.^-1).*(V.^2)).*cos(gamma)) + Cgamma;
    dpsi      = -(V.^-1).*L.*sin(sigma).*(cos(gamma).^-1) - (r.^-1).*V.*cos(gamma).*cos(psi).*tan(phi) + Cpsi;
    dS        =  V.*cos(gamma);
    dsigma    =  sigmad;
    dsigmad   =  param.Kp*(sigmar - sigma) + param.Kd*(sigmadr - sigmad);

    % Control Acceleration Constraint
    dsigmad   = Saturate(dsigmad, param.dsigmadMin, param.dsigmadMax);
    
ode  = [dr; dtheta; dphi; dV; dgamma; dpsi; dS; dsigma; dsigmad];

%------------------------------------
% END: function OptimalBank_Dae.m
%------------------------------------