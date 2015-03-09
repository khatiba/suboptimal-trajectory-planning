function state = CreateState(state, t, y, P)

    global param
    
    st = struct;

    st.P = P;
    
    st.t       = t;
    st.r       = y(:, 1);
    st.theta   = y(:, 2);
    st.phi     = y(:, 3);
    st.V       = y(:, 4);
    st.gamma   = y(:, 5);
    st.psi     = y(:, 6);
    st.S       = y(:, 7);
    st.sigma   = y(:, 8);
    st.sigmad  = y(:, 9);
    

    E0 = 0.5*(st.V(1))^2 - param.mu*st.r(1)^-1;
    Ef = 0.5*(st.V(end))^2 - param.mu*st.r(end)^-1;
    E = 0.5*((st.V).^2) - param.mu*(st.r).^-1;
    st.En = (E - E0) / (Ef - E0);

    h = st.r-param.r_eq;
    [rho Vs] = Atmosph(h);
    M = (st.V)./Vs;
    [CD CL] = AeroCoeff(M);
    D = 0.5*rho.*CD.*(st.V).^2*(param.Area/param.mass);
    L = 0.5*rho.*CL.*(st.V).^2*(param.Area/param.mass);

    st.D = D;
    st.L = L;
    st.h = h;

    g = param.mu*st.r.^-2;
    Cgamma = 2*param.omega.*cos(st.psi).*cos(st.phi);
    
    st.dgamma = (st.V.^-1).*(L.*cos(st.sigma)-(g-(st.r.^-1).*(st.V.^2)).*cos(st.gamma)) + Cgamma;

    [down cross] = DrCr(param.phi0, param.phif, st.phi(end), param.theta0, param.thetaf, st.theta(end), param.r_eq);
    
    st.down = down;
    st.cross = cross;

    state = st;
    
end

