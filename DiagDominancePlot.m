clear all
clc

global param;
InitParams();

Downrange = 720000;
Crossrange = 0;

[thetaf phif] = finalposition(param.theta0, param.phi0, param.psi0, Downrange, Crossrange, param.r_eq);
param.thetaf = thetaf;
param.phif = phif;

state = [];
results = [];
P = [0.3,0.6,0.9];
for i = 0.7:0.1:0.9
    P(1) = i;
    [t, y] = ode45(@Dae, param.tspan, param.ic, param.options, P);
    state = CreateState(state,t,y,P);
    results = [results state];
end

%%
M = zeros(length(results),3);
for i = 1:1:length(results)
    
    state = results(i);
    Pt = state.P(1);

    M(i,1) = state.down/1000;
    M(i,2) = state.cross/1000;
    M(i,3) = state.h(end)/1000;
    Pp(i) = Pt;

end
latex(M);