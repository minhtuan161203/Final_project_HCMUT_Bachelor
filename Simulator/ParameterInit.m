Ts = 0.001;

%% BLDC BLM57180
Rs = 0.9; % stator resistor (ohm)
Ls = 2e-3; % stator inductor (H)
J = 23e-5; %Inertial Moment (kg.m^2)
B = 1e-3;% Viscous damping (N.m.s)
Kt = 0.085; % N.m/A
np = 2; % pole pair
Vdc = 36; % V supply for inverter (V)
psi_m = Kt/(1.5*np); % Flux linkage 
I_max = 20*sqrt(2);   % Max current (A)
I_dm = 6.7;           % Nominal current (A) 
T_dm = 0.57;          % Nominal torque (N.m)
w_dm = 3000;          % Nominal speed (RPM)
w_max = 4000; % Max speed (RPM)

%% Zhongke SY60D04030 Motor
% Rs = 2.4; % stator resistor (ohm)
% Ls = 3.34e-3; % stator inductor (H)
% J = 3.4e-5; %Inertial Moment (kg.m^2)
% B = 3e-3;% Viscous damping (N.m.s)
% Kt = 0.44; % N.m/A
% np = 5; % pole pair
% Vdc = 100; % V supply for inverter (V)
% psi_m = Kt/(1.5*np); % Flux linkage 
% V_dm = 220;    % Nominal voltage (V)
% I_dm = 2;   % Nominal current (A)
% I_max = 3.81;  % Max current (A)
% w_dm = 3000;   % Nominal speed (RPM)
% w_max = 4000;  % Max speed (RPm)
% P = 400;       % Power (W)
% Te_dm = 1.27;  % Nominal torque (kg.m^2)
% Te_max = 3.81; % Max torque (kg.m^2)
%% APM-SB04ADK Motor
% Rs = 2.4; % stator resistor (ohm)
% Ls = 3.34e-3; % stator inductor (H)
% J = 3.4e-5; %Inertial Moment (kg.m^2)
% B = 3e-3;% Viscous damping (N.m.s)
% Kt = 0.44; % N.m/A
% np = 5; % pole pair
% Vdc = 300; % V supply for inverter (V)
% psi_m = Kt/(1.5*np); % Flux linkage 
% V_dm = 200;    % Nominal voltage (V)
% I_dm = 2.89;   % Nominal current (A)
% I_max = 8.67;  % Max current (A)
% w_dm = 3000;   % Nominal speed (RPM)
% w_max = 5000;  % Max speed (RPm)
% P = 400;       % Power (W)
% Te_dm = 1.27;  % Nominal torque (N.m)
% Te_max = 3.81; % Max torque (N.m)
%% Panasonic MSMD022G1S Motor
% Rs = 4; % stator resistor (ohm)
% Ls = 6.55e-3; % stator inductor (H)
% J = 14e-5; %Inertial Moment (kg.m^2)
% B = 7e-3;% Viscous damping (N.m.s)
% Kt = 0.28; % N.m/A
% np = 5; % pole pair
% Vdc = 300; % V supply for inverter (V)
% psi_m = Kt/(1.5*np); % Flux linkage 
% V_dm = 200;    % Nominal voltage (V)
% I_dm = 1.6;   % Nominal current (A)
% I_max = 2.26;  % Max current (A)
% w_dm = 3000;   % Nominal speed (RPM)
% w_max = 5000;  % Max speed (RPm)
% P = 200;       % Power (W)
% Te_dm = 0.64;  % Nominal torque (kg.m^2)
% Te_max = 1.91; % Max torque (kg.m^2)
