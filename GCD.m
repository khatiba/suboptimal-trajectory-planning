function d = GCD(phi1, phi2, theta1, theta2)
d = acos(cos(phi1)*cos(phi2)*cos(theta1 - theta2) + sin(phi1)*sin(phi2));