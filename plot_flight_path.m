clc
clear
close all

% Define the initial constants
X0 = [
    85;
    0;
    0;
    deg2rad(3);
    0;
    -1524;
    ];

u0 = [
    deg2rad(-2);
    0.6];


sim('cessnamodel.slx');

%% Simulation results

t = simX.Time;
X = simX.Data;

%% States

u_vel = X(:,1);
w_vel = X(:,2);
q     = X(:,3);
theta = X(:,4);
PN    = X(:,5);
PD    = X(:,6);

%% Derived quantities

altitude = -PD;
V         = sqrt(u_vel.^2 + w_vel.^2);
alpha     = atan2(w_vel,u_vel);

%% Controls

U = simU.Data;

de  = U(1);
uth = U(2);

%% Flight path

figure
plot(PN,altitude,'LineWidth',2)
grid on
xlabel('North Position, P_N (m)')
ylabel('Altitude, h (m)')
title('Cessna 182 Longitudinal Flight Path')

