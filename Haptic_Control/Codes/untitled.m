% This program is to generate Bode plots using the real recorded data, but
% the model we use is still linear Matlab model
% by Yi Yang
% Date: 9/23/2016
%--------------------------------------------------------------------------
% Admittance Control Devices
clear all; close all;
s = tf('s');
dataID = sysID();
Z_h = dataID.M * s + dataID.B; % the inherent impedance of haptic devices
Kve = 400; % [N/m], virtual environment spring constant
Z_ve = Kve/s; % impedance of virtual environment
C = 2; % portional control law
Z_padm = (dataID.n * C * Z_ve + Z_h)/(dataID.n * C + 1);

% Series Elastic Actuator Device
k = 904; % [N/m], Physical spring constant
kk = k/s + dataID.b;
Z_e = dataID.m * s;
Z_psea = (Z_e * (Z_h + k * dataID.n * C/s + kk) + kk * (Z_h + dataID.n * C * Z_ve))/(Z_h + k * dataID.n * C/s + kk);

% Add sinusoidal signal with white noise of snr = 10 to the linear model
t = linspace(0, 10, 10001); % time step chosen to be ~ 0.001
u0 = sin(2*pi*t);
ui = awgn(u0, 10, 'measured');
fo_adm = lsim(Z_padm, ui, t);
fo_sea = lsim(Z_psea, ui, t);
figure;
plot(t,[fo_adm fo_sea]);