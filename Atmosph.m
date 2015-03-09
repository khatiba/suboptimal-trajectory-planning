function [rho Vs] = Atmosph(h)


    % Atmospheric Density
    
    a0 = -4.324;
    a1 = -9.204E-5;
    a2 = -1.936E-11;
    a3 = -7.507E-15;
    a4 =  4.195E-20;
    a = [a0 a1 a2 a3 a4]';
    
    
    den=0;
    for i=1:5
        den_new = a(i)*h.^(i-1);
        den = den + den_new;
    end
    
    rho = exp(den);
    
    % Speed of sound
    
        
    b0 =  223.8;
    b1 = -0.0002004;
    b2 = -1.588E-8;
    b3 = 1.404E-13;
    b = [b0 b1 b2 b3]';
    
    
    vel=0;
    for i=1:4
        vel_new = b(i)*h.^(i-1);
        vel = vel + vel_new;
    end
    
    Vs = vel;
end
    
   
    