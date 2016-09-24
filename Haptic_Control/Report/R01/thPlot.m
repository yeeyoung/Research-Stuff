% This program is used for obtaining bode plots of linear model
% Admittance Control Devices
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

close all; figure(1);
opts = bodeoptions('cstprefs');
opts.PhaseVisible = 'off';
opts.FreqUnits = 'Hz';
opts.Grid = 'on';
bodemag(Z_padm, opts);
hold on;
bodemag(Z_psea, opts);
bodemag(Z_ve, '--', opts);
legend('Admittance', 'SEA(k= 904)', 'ideal line');
title('Virtual Stiffness');
hold off;
