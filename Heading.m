function heading = Heading(phi1, phi2, theta1, theta2, d12)

    phi = sign(theta2 - theta1)*acos((sin(phi2) - sin(phi1)*cos(d12))/(cos(phi1)*sin(d12)));
    
    if (theta1 == theta2)
        if (phi2-phi1 > 0)
            phi = 0;
        else
            phi = pi;
        end
    end
    
heading = pi/2 - phi;