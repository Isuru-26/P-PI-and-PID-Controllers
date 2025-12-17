clc;
clear;
close all;

%% Motor Parameters
% Define physical parameters of the DC motor
J = 0.01;        % Moment of inertia (kg.m^2)
b = 0.1;         % Damping coefficient (N.m.s)
K = 0.01;        % Motor torque and back EMF constant
R = 1;           % Armature resistance (Ohm)
L = 0.5;         % Armature inductance (H)

%% Transfer Function to Angular Position
num = K;
den = [J*L, (J*R + L*b), (b*R + K^2), 0];
motor_tf_pos = tf(num, den);

disp('DC Motor Transfer Function:');
motor_tf_pos

%% Auto-tuned Controllers
C_P     = pidtune(motor_tf_pos, 'P');  % Proportional controller
C_PI    = pidtune(motor_tf_pos, 'PI'); % Proportional-Integral controller
% Auto-tune PID with a target phase margin between 48 and 51 degrees
options = pidtuneOptions('PhaseMargin', 47); %  Approximate phase margin
C_PID_auto = pidtune(motor_tf_pos, 'PID', options); % Auto-tuned PID
% Display gains
disp('Auto-tuned P Controller:');
disp(C_P);
disp('Auto-tuned PI Controller:');
disp(C_PI);
disp('Auto-tuned PID Controller:');
disp(C_PID_auto);

%% Manually Optimized PID Controller to Reduce Settling Time
% Tuned to reduce settling time and improve response
Kp = C_PID_auto.Kp * 1.1;  % increased proportional gain
Ki = C_PID_auto.Ki * 0.05; % Reduced integral action
Kd = C_PID_auto.Kd * 1.45; % Increased derivative action

% manually tuned PID gains
C_PID = pid(Kp, Ki, Kd);

disp('- Manually Tuned PID Gains for Faster Response -');
disp(C_PID);

%% Closed-loop Transfer Functions
T_P     = feedback(C_P * motor_tf_pos, 1);
T_PI    = feedback(C_PI * motor_tf_pos, 1);
T_PID = feedback(C_PID * motor_tf_pos, 1);  

%% Time and Reference Input
t = 0:0.01:30;  % Simulation time
ref = (pi/2) * ones(size(t));  % Step input to 90 degrees

%% Step Responses to Reference Input
y_P     = lsim(T_P, ref, t);
y_PI    = lsim(T_PI, ref, t);
y_PID = lsim(T_PID, ref, t);

% Convert radian outputs to degrees
y_P_deg   = rad2deg(y_P);
y_PI_deg  = rad2deg(y_PI);
y_PID_deg = rad2deg(y_PID);

%% Disturbance at t = 2s
dist = zeros(size(t));
dist(t >= 2) = 0.075;  % Disturbance of 0.075 rad after 2s

y_ref_P     = lsim(T_P, ref, t);
y_dist_P    = lsim(1 - T_P, dist, t);
y_P_total   = y_ref_P + y_dist_P;

y_ref_PI    = lsim(T_PI, ref, t);
y_dist_PI   = lsim(1 - T_PI, dist, t);
y_PI_total = y_ref_PI + y_dist_PI;

y_ref_PID   = lsim(T_PID, ref, t);
y_dist_PID  = lsim(1 - T_PID, dist, t);
y_PID_total = y_ref_PID + y_dist_PID;

%% Plot: P Controller with Disturbance
figure;
plot(t, rad2deg(y_P_total), 'b', 'LineWidth', 2);
yline(90, '--b', 'Target');
ylim([0 110]);
xlim([0 10]);
title('P Controller with Disturbance at t=2s');
xlabel('Time (s)'); ylabel('Position (°)');
grid on;

%% Plot: PI Controller with Disturbance
figure;
plot(t, rad2deg(y_PI_total), 'b', 'LineWidth', 2);
yline(90, '-b', 'Target');
ylim([0 110]);
xlim([0 10]);
title('PI Controller with Disturbance at t=2s');
xlabel('Time (s)'); ylabel('Position (°)');
grid on;

%% Plot: PID Controller with Disturbance
figure;
plot(t, rad2deg(y_PID_total), 'b', 'LineWidth', 2);
yline(90, '--b', 'Target');
ylim([0 110]);
xlim([0 10]);
title('PID Controller with Disturbance at t=2s');
xlabel('Time (s)'); ylabel('Position (°)');
grid on;

%% Compare All Controllers
figure;
plot(t, y_P_deg, 'g', t, y_PI_deg, 'b', t, y_PID_deg, 'r', 'LineWidth', 1.5);
yline(90, 'k--', 'Target');
ylim([0 110]);
xlim([0 10]);
legend('P', 'PI', 'PID', 'Target');
title('Step Response Comparison');
xlabel('Time (s)'); ylabel('Position (°)');
grid on;

%% P Controller
info_p = stepinfo(y_P_deg, t, 90);
disp('- Step response for P Controller -');
disp(info_p);

figure;
plot(t, y_P_deg, 'g', 'LineWidth', 1); hold on;
ylim([0 110]);
xlim([0 10]);
yline(90, '--k', '90°', 'LineWidth', 0.75);
xline(info_p.RiseTime, '--r', sprintf('Rise Time = %.2fs', info_p.RiseTime));
xline(info_p.SettlingTime, '--b', sprintf('Settling Time = %.2fs', info_p.SettlingTime));
yline(info_p.Peak, '--m', sprintf('Overshoot = %.2f%%', info_p.Overshoot));
title('P Controller Step Response');
xlabel('Time (s)'); ylabel('Position (°)');
legend('P Response','Rise Time','Overshoot','Settling Time','90°');
grid on;

%% PI Controller
info_pi = stepinfo(y_PI_deg, t, 90);
disp('- Step response for PI Controller -');
disp(info_pi);

figure;
plot(t, y_PI_deg, 'b', 'LineWidth', 1); hold on;
ylim([0 120]);
yline(90, '--k', '90°', 'LineWidth', 0.75);
if ~isnan(info_pi.RiseTime)
    xline(info_pi.RiseTime, '--r', sprintf('Rise Time = %.2fs', info_pi.RiseTime));
end
if ~isnan(info_pi.SettlingTime)
    xline(info_pi.SettlingTime, '--b', sprintf('Settling Time = %.2fs', info_pi.SettlingTime));
end
if ~isnan(info_pi.Peak)
    yline(info_pi.Peak, '--m', sprintf('Overshoot = %.2f%%', info_pi.Overshoot));
end
title('PI Controller Step Response');
xlabel('Time (s)'); ylabel('Position (°)');
legend('P Response','Rise Time','Overshoot','Settling Time','90°');
grid on;

%% PID Controller
info_pid = stepinfo(y_PID_deg, t, 90);
disp('- Step response for PID Controller -');
disp(info_pid);

figure;
plot(t, y_PID_deg, 'r', 'LineWidth', 1); hold on;
ylim([0 110]);
xlim([0 10]);
yline(90, '--k', '90°', 'LineWidth', 0.75);
xline(info_pid.RiseTime, '--r', sprintf('Rise Time = %.2fs', info_pid.RiseTime));
xline(info_pid.SettlingTime, '--b', sprintf('Settling Time = %.2fs', info_pid.SettlingTime));
yline(info_pid.Peak, '--m', sprintf('Max Overshoot = %.2f%%', info_pid.Overshoot));
title('PID Controller Step Response');
xlabel('Time (s)'); ylabel('Position (°)');
legend('P Response','Rise Time','Max Overshoot','Settling Time','90°');
grid on;