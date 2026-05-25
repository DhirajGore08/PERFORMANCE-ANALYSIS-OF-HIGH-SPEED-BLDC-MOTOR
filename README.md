# PERFORMANCE-ANALYSIS-OF-HIGH-SPEED-BLDC-MOTOR (FOC Based Speed controller)
 Developed and simulated a high-speed BLDC motor in MATLAB/Simulink using Field-Oriented Control (FOC) and analyzed its speed, torque, current, power, and efficiency performance.
 
clc;
clear;
close all;

%% ================= SYSTEM LIMITS =================
V_supply = 48;
RPM_MAX = 50000;

%% ================= REFERENCE =================
N_ref = 50000;
omega_ref = 2*pi*N_ref/60;

%% ================= MOTOR PARAMETERS =================
Ke = 0.0042;
% Min : 0.0035
% Max : 0.0060

Kt = 0.024;              % Increased torque constant
% Min : 0.015
% Max : 0.035

R_phase = 0.18;          % Optimized resistance
% Min : 0.05 Ω
% Max : 0.30 Ω

J = 1.2e-5;
% Min : 5e-6
% Max : 2e-5

B = 1e-6;                % Reduced friction
% Min : 5e-7
% Max : 5e-6

T_load = 0.005;
% Min : 0.002 Nm
% Max : 0.010 Nm


I_max = 12;
% Min : 5 A
% Max : 20 A

%% ================= FOC PARAMETERS =================
Ld = 0.00015;
% Min : 0.00005 H
% Max : 0.0005 H

Lq = 0.00015;
% Min : 0.00005 H
% Max : 0.0005 H


PolePairs = 2;
% Min : 1
% Max : 8

Flux_PM = Ke;
% Min : 0.0035
% Max : 0.0060

%% ================= SIMULATION =================
dt = 1e-4;
t_end = 2;

t = 0:dt:t_end;

%% ================= VARIABLES =================
omega = zeros(size(t));

id = zeros(size(t));
iq = zeros(size(t));

vd = zeros(size(t));
vq = zeros(size(t));

i = zeros(size(t));

V_control = zeros(size(t));

%% ================= DISPLAY =================
fprintf('\nTime\tSpeed(RPM)\tVoltage\n');
fprintf('---------------------------------\n');

%% ================= FOC CONTROL LOOP =================
for k = 1:length(t)-1

    %% Electrical Speed
    omega_e = PolePairs * omega(k);

    %% Dynamic Torque Current Reference
    speed_ratio = omega(k) / omega_ref;

    if speed_ratio < 0.20

        iq_ref = 10;

    elseif speed_ratio < 0.50

        iq_ref = 7;

    elseif speed_ratio < 0.75

        iq_ref = 5;

    elseif speed_ratio < 0.90

        iq_ref = 3.5;

    elseif speed_ratio < 0.97

        iq_ref = 2.6;

    else

        iq_ref = 1.8;

    end

    %% Current Limiting
    if iq_ref > I_max
        iq_ref = I_max;
    end

    %% ================= FOC VOLTAGE EQUATIONS =================
    vd(k) = -omega_e * Lq * iq(k);

    vq(k) = R_phase * iq_ref ...
          + omega_e * Flux_PM;

    %% Voltage Magnitude
    V_mag = sqrt(vd(k)^2 + vq(k)^2);

    %% Voltage Saturation
    if V_mag > V_supply

        scale = V_supply / V_mag;

        vd(k) = vd(k) * scale;
        vq(k) = vq(k) * scale;

        V_mag = V_supply;

    end

    V_control(k) = V_mag;

    %% ================= CURRENT DYNAMICS =================
    did = (vd(k) ...
          - R_phase * id(k) ...
          + omega_e * Lq * iq(k)) / Ld;

    diq = (vq(k) ...
          - R_phase * iq(k) ...
          - omega_e * (Ld * id(k) + Flux_PM)) / Lq;

    %% Current Update
    id(k+1) = id(k) + did * dt;

    iq(k+1) = iq(k) + diq * dt;

    %% ================= CURRENT LIMITING =================
    i_mag = sqrt(id(k+1)^2 + iq(k+1)^2);

    if i_mag > I_max

        scale_i = I_max / i_mag;

        id(k+1) = id(k+1) * scale_i;
        iq(k+1) = iq(k+1) * scale_i;

        i_mag = I_max;

    end

    i(k) = i_mag;

    %% ================= ELECTROMAGNETIC TORQUE =================
    T_e = Kt * iq(k);

    %% ================= MECHANICAL DYNAMICS =================
    domega = (T_e ...
             - T_load ...
             - B * omega(k)) / J;

    omega(k+1) = omega(k) + domega * dt;

    %% Speed Limiting
    if omega(k+1) > omega_ref
        omega(k+1) = omega_ref;
    end

    %% Prevent Negative Speed
    if omega(k+1) < 0
        omega(k+1) = 0;
    end

    %% ================= LIVE DISPLAY =================
    if mod(k,200) == 0

        fprintf('%.2f\t%.0f\t\t%.2f\n', ...
        t(k), omega(k)*60/(2*pi), V_control(k));

    end

end

%% ================= FINAL VALUES =================
i(end) = i(end-1);

V_control(end) = V_control(end-1);

%% ================= RESULTS =================
N = omega * 60/(2*pi);

E = Ke * omega;

T = Kt * iq;

%% ================= POWER =================
P_out = T .* omega;

%% Copper Loss
P_copper = (i.^2) * R_phase;

%% Mechanical Loss
P_mech_loss = B * (omega.^2);

%% Switching + Core Loss
P_misc = 3;

%% Corrected Input Power
P_in = P_out + P_copper + P_mech_loss + P_misc;

%% ================= EFFICIENCY =================
eff = (P_out ./ P_in) * 100;

eff(isnan(eff)) = 0;
eff(isinf(eff)) = 0;

eff(eff > 100) = 100;

%% ================= FINAL OUTPUT =================
fprintf('\n========== FINAL RESULTS ==========\n');

fprintf('Reference Speed       : %.0f RPM\n', N_ref);

fprintf('Final Speed           : %.0f RPM\n', N(end));

fprintf('Final Voltage         : %.2f V\n', V_control(end));

fprintf('Final Current         : %.2f A\n', i(end));

fprintf('Back EMF              : %.2f V\n', E(end));

T_final = T(end);

Pout_final = P_out(end);

Pin_final = P_in(end);

fprintf('Torque                : %.4f Nm\n', T_final);

fprintf('Mechanical Power Out  : %.2f W\n', Pout_final);

fprintf('Electrical Power In   : %.2f W\n', Pin_final);

%% Final Efficiency
if Pin_final > 0

    eff_final = (Pout_final / Pin_final) * 100;

else

    eff_final = 0;

end

fprintf('Efficiency            : %.2f %%\n', eff_final);

fprintf('===================================\n');

%% ================= PLOTS =================
figure('Name','FOC Controlled BLDC Motor','NumberTitle','off');

subplot(3,2,1);
plot(t, N,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Speed (RPM)');
title('Speed vs Time');
grid on;

subplot(3,2,2);
plot(t, T,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Torque vs Time');
grid on;

subplot(3,2,3);
plot(t, E,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Back EMF (V)');
title('Back EMF vs Time');
grid on;

subplot(3,2,4);
plot(t, eff,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Efficiency (%)');
title('Efficiency vs Time');
ylim([0 100]);
grid on;

subplot(3,2,5);
plot(t, V_control,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Voltage (V)');
title('FOC Voltage vs Time');
grid on;

subplot(3,2,6);
plot(t, iq,'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Iq Current (A)');
title('Torque Producing Current');
grid on;
