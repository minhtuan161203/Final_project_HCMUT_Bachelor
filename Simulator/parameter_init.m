Rs = 0.9; % stator resistor (ohm)
Ls = 2e-3; % stator inductor (H)
J = 23e-5; %Inertial Moment (kg.m^2)
B = 0.00003;% Viscous damping (N.m.s)
Kt = 0.085; % N.m/A
np = 2; % pole pair
Vdc = 36; % V supply for inverter (V)
psi_m = Kt/(1.5*np); % Flux linkage 