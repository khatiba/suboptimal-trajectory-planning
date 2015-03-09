function [down cross] = DrCr(phi1, phi2, phi3, theta1, theta2, theta3, r_eq)

    % Distance from initial to actual final position
    d12 = GCD(phi1, phi2, theta1, theta2);
    d13 = GCD(phi1, phi3, theta1, theta3);

    % Bearing from initial to target point
    psi12 = Heading(phi1, phi2, theta1, theta2, d12);
    % Bearing from initial to actual final point
    psi13 = Heading(phi1, phi3, theta1, theta3, d13);
    
    % Crossrange distance
    Cr = asin(sin(d13)*sin(psi12-psi13));

    % Downrange distance
    Dr = acos(cos(d13)/cos(Cr));

down = (Dr - d12)*r_eq;
cross = Cr*r_eq;