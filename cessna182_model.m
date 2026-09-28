function xdot = cessna182_model(x,u)

% CESSNA182_MODEL Longitudinal 3DOF Equations of Motion

% States:x = [u; w; q; theta; PN; PD]
% Controls:u = [de; uth]   (elevator deflection [rad], throttle [0-1])


%------------------------------- NOMINAL VEHICLE CONSTANTS-------------------------------
m     = 1202;                % Aircraft mass (kg)
cbar  = 1.49;                % Aerodynamic Chord (m)
S     = 16.2;                % Wing planform area (m^2)
b     = 11;                  % Wing span (m)


Iyy   = 1825;                % kg*m^2

g0    = 9.80665;             % m/s^2, sea-level standard gravity
Re    = 6371000;             % m, mean Earth radius (for g(h) inverse-square law)

% Aerodynamic derivatives
CD0     = 0.0270;
CDa     = 0.121;
CDde    = 0;
CL0     = 0.307;
CLa     = 4.41;
CLadot  = 1.7;
CLq     = 3.9;
CLde    = 0.43;
Cm0     = 0.04;
Cma     = -0.613;
Cmadot  = -7.27;
Cmq     = -12.4;
Cmde    = -1.122;

% Propulsion 
Pmax    = 230*745.7;              % Max shaft power conert to SI  (W)
eta_p   = 0.80;                  % Propeller efficiency (assumed, literature)

%---------------------------------STATE AND CONTROL VECTORS------------------------------------------
u_vel = x(1);       % body axis x velocity
w_vel = x(2);       % body axis z velocity
q     = x(3);       % pitch rate
theta = x(4);       % pitch angle
PN    = x(5);       % north position
PD    = x(6);       % down position 

de    = u(1);        % elevator deflection 
uth   = u(2);         % throttle setting 

%--------------------------------AIR DATA CALCULATIONS---------------------------------------------------
                           
V     = sqrt(u_vel^2 + w_vel^2);
alpha = atan2(w_vel,u_vel);

h     = -PD;
rho   = isa_density(h);      % ISA atmosphere density (see local function below)
qbar  = 0.5*rho*V^2;         % dynamic pressure

qhat  = q*cbar/(2*V);

% Altitude varying gravity 
g     = g0*(Re/(Re+h))^2;

%------------------------AERODYNAMIC COEFFICIENTS, ALPHA DOT EXCLUDED  PART----------------------
% Split CL, Cm into a part independent of alpha-dot ("base") and the
% coefficient multiplying alpha_dot (adothat = alpha_dot*cbar/(2V)).
CL_base = CL0 + CLa*alpha + CLq*qhat + CLde*de;
Cm_base = Cm0 + Cma*alpha + Cmq*qhat + Cmde*de;
CD      = CD0 + CDa*alpha + CDde*de;                     

L_base  = qbar*S*CL_base;
D       = qbar*S*CD;
Lcoef   = qbar*S*CLadot*cbar/(2*V);           %   dL/d(alpha_dot)
Mcoef   = qbar*S*cbar*Cmadot*cbar/(2*V);     %   dMy/d(alpha_dot)

My_base = qbar*S*cbar*Cm_base;

%------------------------PROPULSION FORCE------------------------------------------
SP = uth*Pmax;                             % shaft power 
T  = eta_p*SP/V;

% ROTATION MATRICES 
C_wb = [ cos(alpha), 0, -sin(alpha);
                  0, 1,           0;
         sin(alpha), 0,  cos(alpha)];

% Aerodynamic force in wind axes to convert to body axes 
F_wind_base = [-D; 0; -L_base];
F_body_base = C_wb*F_wind_base;

% d(F_aero_body)/d(alpha_dot)
dF_body_dadot = C_wb*[0;0;-Lcoef];

% Gravity rotation
Cbv = [ cos(theta), 0, -sin(theta);
                  0, 1,           0;
         sin(theta), 0,  cos(theta)];

Fg_v = [0; 0; m*g];
Fg_b = Cbv*Fg_v;

%--------------------------------TOTAL FORCE in Body Axes, No alpha-dot term---------------
F_thrust_b = [T; 0; 0];

F_total_base = F_body_base + F_thrust_b + Fg_b;   % alpha-dot not included !!
Fx_base = F_total_base(1);
Fz_base = F_total_base(3);
Fx_coef = dF_body_dadot(1);  
Fz_coef = dF_body_dadot(3);   

%-------------------------------TOTAL MOMENT in Body Axes, No alpha-dot term---------------------------
My_coef = Mcoef;


% 3x3 linear system Solving coupled alpha-dot implicit system

%   udot = Fx_base/m - q*w_vel + (Fx_coef/m)*alpha_dot          ... (1)
%   wdot = Fz_base/m + q*u_vel + (Fz_coef/m)*alpha_dot          ... (2)
%   alpha_dot*V^2 = u_vel*wdot - w_vel*udot                     ... (3)

A1 = Fx_base/m - q*w_vel;
B1 = Fx_coef/m;
A2 = Fz_base/m + q*u_vel;
B2 = Fz_coef/m;

% [ 1     0    -B1 ] [udot     ]   [A1]
% [ 0     1    -B2 ] [wdot     ] = [A2]
% [-w_vel u_vel -V^2] [alpha_dot]   [0]
Mmat = [1,      0,      -B1;
        0,      1,      -B2;
       -w_vel,  u_vel,  -V^2];
rhs  = [A1; A2; 0];

sol       = Mmat\rhs;
udot      = sol(1);
wdot      = sol(2);
alpha_dot = sol(3);

% Final My including the alpha_dot contribution
My = My_base + My_coef*alpha_dot;

% Rotational simplified  
qdot = My/Iyy;

% Angular position simplified for phi=0
thetadot = q;

% Translational position simplified for phi=0,psi=0
PNdot =  u_vel*cos(theta) + w_vel*sin(theta);
PDdot = -u_vel*sin(theta) + w_vel*cos(theta);

% OUTPUT
xdot = [udot; wdot; qdot; thetadot; PNdot; PDdot];

end

%======================================================================
function rho = isa_density(h)
% Simple ISA troposphere model (valid up to 11000 m)
    rho0 = 1.225;      
    T0   = 288.15;     
    L    = 0.0065;     
    R    = 287.05;     
    g0   = 9.80665;   

    T   = T0 - L*h;
    rho = rho0*(T/T0)^(g0/(R*L)-1);
end