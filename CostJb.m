function J = CostJb( P )

    global param

    st = [];
    [t, y] = ode45(@Dae, param.tspan, param.ic, param.options, P);
    st = CreateState(st,t,y,P);

    hf = st.h(end);
    gf = st.gamma(end);
   
J = param.kg*(gf).^2 - param.kh*hf;