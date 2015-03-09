%------------------------------------
% BEGIN: Main
%------------------------------------
clear all; close all; clc;

global param

state = struct([]);
results = struct([]);

% Create and store all constants and options in the param variable
param = struct;
% Initial Conditions
param.r0      =  3520760;
param.theta0  = -90.072   *(pi/180);
param.phi0    = -43.898   *(pi/180);
param.V0      =  5505;
param.gamma0  = -14.15    *(pi/180);
param.psi0    =  4.99     *(pi/180);
param.S0      =  0;
param.sigma0  =  10       *(pi/180);
param.sigmad0 =  0        *(pi/180);
param.t0      =  0;

% Planet Constants (Mars)
param.mu    = 4.284E13;     % Gravitational Constant
param.r_eq  = 3396201;      % Equatorial Radius
param.omega = 7.095E-5;     % Planet Angular Rate

% Vehicle Constant (MSL)
param.Area  = 15.9;         % Surface Area
param.mass  = 2804;         % Mass

% Control constraints
param.sigmarMin   =  10   *(pi/180);
param.sigmarMax   =  80   *(pi/180);
param.sigmadMin   = -20   *(pi/180);
param.sigmadMax   =  20   *(pi/180);
param.dsigmadMin  = -10    *(pi/180);
param.dsigmadMax  =  10    *(pi/180);

% Final Target States
param.Vf         = 450;                    % Final desired velocity
param.rf         = 8000 + param.r_eq;      % Final minimum radius from center of planet

% Initial and Final Energy
param.E0    = 0.5*(param.V0)^2 - param.mu*param.r0^-1;  % Initial energy at entry point
param.Ef    = 0.5*param.Vf^2 - param.mu*param.rf^-1;    % Final energy at desired parachute deployment point

% ODE45 Initial Conditions, Time Span and Options
param.ic = [param.r0; param.theta0; param.phi0; param.V0;...
            param.gamma0; param.psi0; param.S0;...
            param.sigma0; param.sigmad0];
        
param.options   = odeset('Events', @IntEvents, 'AbsTol',1e-5, 'RelTol',1e-5);
param.tspan     = [param.t0; inf]; % Time span to integrate

% FBL Tracking gains
param.kh = 1;
param.kg = -5e-6;

param.Cost(1)   = 0;

% Control dynamics parameters
param.Kp  = 0.6; % Proportional gain
param.Kd  = 2.2*(param.Kp)^0.5;  % Differential gain

% % initial guesses for Pdr, Pcr, Phf
% PG = [0.25  0.71  0.96; % 700k
%       0.31  0.73  0.96; % 710k
%       0.36  0.75  0.96; % 720k
%       0.406 0.765 0.96; % 730k
%       0.45  0.77  0.96  % 740k
%       ];

% PG = [0.25  0.71  0.96; % 700k
%       0.36  0.75  0.96; % 720k
%       0.45  0.77  0.96  % 740k
%       ];

PG1 = [0.208  0.663  0.934]; % 700km
PG2 = [0.27  0.685  0.94]; % 710km
PG3 = [0.32  0.70  0.94]; % 720km
PG4 = [0.373  0.727  0.943]; % 730km
PG5 = [0.41  0.74  0.94]; % 740km

PG = [PG1; PG2; PG3; PG4; PG5];

% PG = [0.15 0.68 0.96]; % 680km
% PG = [0.45 0.77 0.96]; % 740km
% PG = [0.66 0.83 0.96]; % 800km

nn = 1; count = 1;
for downrange = 700000:10000:740000
% for downrange = 740000
    crossrange = 0;

    % Final position in theta and phi given downrange and crossrange input
    [thetaf phif] = Target(param.theta0, param.phi0, param.psi0, downrange, crossrange, param.r_eq);
    param.thetaf = thetaf;
    param.phif = phif;
    
    % Initial guesses
    param.Pdr(1) = PG(nn,1);
    param.Pcr(1) = PG(nn,2);
    param.Phf(1) = PG(nn,3);

    while count

        % Create the parameter array for integration
        P = [param.Pdr(count) param.Pcr(count) param.Phf(count)];
        % ODE45 Integration of Vehicle Dynamics
        [t, y] = ode45(@Dae, param.tspan, param.ic, param.options, P);

        % Store the state structure globally
        state = CreateState(state, t, y, P);

        
        % Start Newton's method
        dP = 0.05;

        % Array of parameters
        Pi = [param.Pdr(count) param.Pcr(count) param.Phf(count)]';
        param.Cost(count) = Cost(Pi);

        J = ones(length(Pi),3);
        % Loop through the three perturbations of parameters
        for i = 1:1:length(Pi)
            P = Pi;
            c = 1;
            for k = -1:1:1
                % Perturb each parameter backwards and forwards
                P(i) = Pi(i) + k*dP;
                J(i,c) = Cost(P);
                c = c + 1;
            end
        end
 
        % Take differences between columns
        dJ  = diff(J,1,2);
        ddJ = diff(J,2,2);

        % If the parameter is on the edge
        [r ~] = find(dJ(:,2) == 0);
        if (isempty(r))
            % Take backward derivative
            dJ = dJ(:,1);
        else
           % Take forward derivative
            dJ = dJ(:,2); 
        end
        
        state.down
        state.cross

        if (abs(state.down) < 1e3 && ...
            abs(state.cross) < 1e3 && ...
            abs(state.gamma(end)) < 20*(pi/180))

            state.cost = param.Cost(count);
            results = [results state];
            nn = nn + 1;
            break;
        end
        
        % Scale the step and check for NaN's
        step = nanCheck(-2e-3*dJ./ddJ);
        param.Pdr(count+1) = Saturate(param.Pdr(count) + step(1), 0, 1);
        param.Pcr(count+1) = Saturate(param.Pcr(count) + step(2), 0, 1);
        param.Phf(count+1) = Saturate(param.Phf(count) + step(3), 0, 1);
        count = count + 1;
    end
end

%% Plots

c = length(results);
colors = varycolor(c);

for i=1:1:c
    
    state = results(i);
    col = colors(i,:);
    
    r        = state.r;
    phi      = state.phi;
    V        = state.V;
    gamma    = state.gamma;
    dgamma   = state.dgamma;
    psi      = state.psi;
    S        = state.S;
    sigma    = state.sigma;
    sigmadot = state.sigmad;
    Enorm    = state.En;
    h        = state.h;
    
    % Bank angle
    subplot(2,2,1);
    pp = plot(Enorm,sigma*(180/pi),'Color',col); grid on; hold on;
    xlabel('Normalized Energy');
    ylabel('Bank Angle [deg]');
    axis([0 1 -90 90]);
    set(pp,'LineWidth',1.1);
    
    % Altitude
    subplot(2,2,2);
    pp = plot(Enorm,h/1000,'Color',col); grid on; hold on;
    xlabel('Normalized Energy');
    ylabel('Altitude [km]');
    axis([0 1 0 50]);
    set(pp,'LineWidth',1.1);
    
%     % Flight Path Angle
%     subplot(2,2,3);
%     pp = plot(Enorm,gamma*(180/pi),'Color',col); grid on; hold on;
%     xl = xlabel('Normalized Energy');
%     yl = ylabel('Flight Path Angle [deg]');
%     axis([0 1 -20 3]);
%     set(pp,'LineWidth',1.1);

%     % FPA Rate
%     subplot(2,2,4);
%     pp = plot(Enorm,dgamma*(180/pi),'Color',col); grid on; hold on;
%     xlabel('Normalized Energy');
%     ylabel('FPA Rate [deg/s]');
%     axis([0 1 -0.4 0.4])
%     set(pp,'LineWidth',1.1);
    
end

%% Table

Data = [];
for i = 1:1:length(results)
    sol = results(i);
    c = i;
    cost = Cost(sol.P);
    % Final altitude
    Data(c,1) = sol.h(end)/1000;
    % Downrange
    Data(c,2) = sol.down;
    % Crossrange
    Data(c,3) = sol.cross;
    % Final flight path angle
    Data(c,4) = sol.gamma(end)*(180/pi);
    % Cost
    Data(c,5) = vpa(cost, 4);
end
Data = sortrows(Data, 1);
latex(Data, 'nomath')

%% Parachute box plot
close all;

points = [3.5 300;
          8   300;
          17  480;
          7   480;
          3.5 410;
          3.5 300];
      
line = [5.55 450;
        15.45  450];

% pp = plot(points(:,2),points(:,1), 'b', line(:,2),line(:,1),'r'); grid on; hold on;
pp = plot(points(:,2),points(:,1), 'b'); grid on; hold on;
xlabel('Velocity [m/s]');
ylabel('Altitude [km]');
axis([280 500 2 18])
 set(pp,'LineWidth',1.1);
