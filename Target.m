function [thetaF,phiF] = Target(theta0, phi0, psi0, down, cross, rp)
    
    A = cos(down/rp);
    B = sin(cross/rp);
    LF = acos(A*cos(asin(B)));
    chi = asin(B/sin(LF));
    eta = chi - psi0 + pi/2;
    phiF = asin(cos(eta)*cos(phi0)*sin(LF)+sin(phi0)*cos(LF));
    inclong = asin(sin(eta)*sin(LF)/cos(phiF));
    thetaF = inclong + theta0;

end