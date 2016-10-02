%% This is the program to apply experimental data to our linear matlab model 
% estimate the transfer function
% by Yi Yang
% Date 9/26/2016
%---------------------------------------------------------------------------
% Admittance Control Devices
clear all; close all;
load('SEA_Free_K334_kp10.mat');
s = tf('s');
dataID = sysID();
Z_h = dataID.M * s + dataID.B; % the inherent impedance of haptic devices
Kve = 0; % [N/m], virtual environment spring constant
Z_ve = Kve/s; % impedance of virtual environment
C = 10; % portional control law
Z_padm = (dataID.n * C * Z_ve + Z_h)/(dataID.n * C + 1);

% Series Elastic Actuator Device
k = 334; % [N/m], Physical spring constant
kk = k/s + dataID.b;
Z_e = dataID.m * s;
Z_psea = (Z_e * (Z_h + k * dataID.n * C/s + kk) + kk * (Z_h + dataID.n * C * Z_ve))/(Z_h + k * dataID.n * C/s + kk);

% This is the free space simulation using the data acquired from SEA_Free_K334_kp10.mat
format long;
Ts=(timeStamp(end - 1)-timeStamp(1))/(length(timeStamp)-2)/(10^6);
LinTime = timeStamp(1)/(10^6):Ts:timeStamp(end - 1)/(10^6);
Resample_RevPosB = interp1(timeStamp(1:end-1)/(10^6), RevPosB(1:end-1), LinTime);
Fuser = k * (Resample_RevPosB)/1000.0; % [N] ([N/m]*[m])
Uo = lsim(1/Z_psea, Fuser, LinTime);
figure(4);
hold on;
plot(LinTime, Fuser, 'b--');
plot(LinTime, Uo, 'r');
legend('Force', 'Velocity');
title('Simulation');
grid on;
box on;
hold off;

% estimate the transfer function
Fs = 1/Ts;
NFFT = 2^nextpow2(length(LinTime));
[Z_e_sea, freq2] = tfestimate(Uo, Fuser, [], [], NFFT, Fs);

[MagTh_sea, PhaseTh_sea, FreqTh_sea] = bode(Z_psea, {2*pi*1e-1, 2*pi*1e3});
Mag_sea = zeros(length(MagTh_sea), 1);
Freq_sea = FreqTh_sea./(2*pi);
for i = 1:length(MagTh_sea)
    Mag_sea(i, 1) = MagTh_sea(1, 1, i);
    Mag_sea(i, 1) = 20 * log10(Mag_sea(i, 1));
end
figure(5);
% bodemag(Z_ve, '--', opts);
hold on;
semilogx(Freq_sea, Mag_sea, 'r');
semilogx(freq2, mag2db(abs(Z_e_sea)));
set(gca,'xscale','log');
legend('the', 'exp');
title('SEA');
grid on;
box on;
hold off;