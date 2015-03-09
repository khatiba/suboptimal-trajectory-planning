function J = Cost( P )

    global param

    st = [];
    
    [t, y] = ode45(@Dae, param.tspan, param.ic, param.options, P);
    st = CreateState(st,t,y,P);

    hf = st.h(end);
    gf = st.gamma(end);
    
    Q = [500^2 500^2 (15*(pi/180))^2 9e4]';
    V = [st.down^2 st.cross^2 gf^2 -hf];

    J = V*(Q.^-1);

end