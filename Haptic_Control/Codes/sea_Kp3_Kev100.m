%% Experiment data demonstration for SEA
% by Yi Yang
% Date: 10/01/2016
% data set: sea_Kp=-3_Kimg=400
%--------------------------------------------------------------------------
clear all;
close all;
struct = importdata('test/20160929114713_sea_Kp=-3_Kimg=400.txt');
data = struct.data(:,:);
timeStamp = data(2:end, 1);
LoadCellForceA = data(2:end, 2);
MotorOutputA = data(2:end, 3);
RevPosA = data(2:end, 4);
RevPosB = data(2:end, 5);
CurrentA = data(2:end, 6); 

% Construct SEA and ADM theoretical model
s = tf('s');
dataID = sysID();
Z_h = dataID.M * s + dataID.B; % the inherent impedance of haptic devices
Kve = 400; % [N/m], virtual environment spring constant
Z_ve = Kve/s; % impedance of virtual environment
C = -3; % portional control law
Z_padm = (dataID.n * C * Z_ve + Z_h)/(dataID.n * C + 1);

% Series Elastic Actuator Device
k = dataID.k; % [N/m], Physical spring constant
kk = k/s + dataID.b;
Z_e = dataID.m * s;
Z_psea = (Z_e * (Z_h + k * dataID.n * C/s + kk) + kk * (Z_h + dataID.n * C * Z_ve))/(Z_h + k * dataID.n * C/s + kk);

% Model constructed from experiment data
format long;
Ts=(timeStamp(end)-timeStamp(1))/(length(timeStamp)-1)/(10^5);
LinTime = timeStamp(1)/(10^5):Ts:timeStamp(end)/(10^5);
LinTime = LinTime';
Resample_RevPosA = interp1(timeStamp(1:end)/(10^5), RevPosA(1:end), LinTime);
Resample_RevPosB = interp1(timeStamp(1:end)/(10^5), RevPosB(1:end), LinTime);
Fuser = k * (Resample_RevPosB)/1000.0; % [N] ([N/m]*[m])
% Uo = lsim(1/Z_psea, Fuser, LinTime);
% First order Euler backward difference methods to approximate velocity
V = diff([Resample_RevPosA(1)+Resample_RevPosB(1); Resample_RevPosA+Resample_RevPosB])./diff([LinTime(1) - Ts; LinTime])/1000;
figure(1);
plot(LinTime, Fuser);
legend('Fuser');
figure(2);
plot(LinTime, Resample_RevPosA+Resample_RevPosB);
legend('RevPosA+RevPosB');
figure(3);
plot(LinTime, V);
legend('Velocity');

% estimate the transfer function
Fs = 1/Ts;
NFFT = 2^nextpow2(length(LinTime));
[Z_e_sea, freq_e] = tfestimate(V, Fuser, [], [], NFFT, Fs);

[MagTh_sea, PhaseTh_sea, FreqTh_sea] = bode(Z_psea, {2*pi*3e-1, 2*pi*2e4});
Mag_sea = zeros(length(MagTh_sea), 1);
Freq_sea = FreqTh_sea./(2*pi);
for i = 1:length(MagTh_sea)
    Mag_sea(i, 1) = MagTh_sea(1, 1, i);
    Mag_sea(i, 1) = 20 * log10(Mag_sea(i, 1));
end
figure(4);
% bodemag(Z_ve, '--', opts);
hold on;
semilogx(Freq_sea, Mag_sea, 'r');
semilogx(freq_e, mag2db(abs(Z_e_sea)));
set(gca,'xscale','log');
legend('the', 'exp');
title('SEA');
grid on;
box on;
hold off;

Pos_des = Fuser/Kve;
V_des = diff([Pos_des(1); Pos_des])./diff([LinTime(1)-Ts; LinTime]);
figure(5);
% plot(V_des, V);
[V_des_sorted, sortID] = sort(V_des);
V_sorted = V(sortID);
plot(V_des_sorted, V_sorted, 'r-');
xlabel('$\dot{x}_{cmd}$');
ylabel('$\dot{x}_{act}$');

% figure(4);
% hold on;
% plot(LinTime, Fuser, 'b--');
% plot(LinTime, Uo, 'r');
% legend('Force', 'Velocity');
% title('Simulation');
% grid on;
% box on;
% hold off;