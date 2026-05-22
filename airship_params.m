%% 
%% airship_params.m
% Physical parameters for the WVU custom airship platform.
% All values sourced directly from Sponaugle (2025) MSME thesis, Chapter 6.
% Load this file before running build_linear_ss.m or run_simulation.m.
 
clear; clc;
 
%% ── Constants ───────────────────────────────────────────────────────────────
g_acc  = 9.81;          % gravitational acceleration [m/s^2]
rho    = 1.225;         % air density at standard conditions [kg/m^3]
 
%% ── Geometry (Table 6.2) ────────────────────────────────────────────────────
a = 0.918;              % semi-major axis of ellipsoidal balloon [m]
b = 0.372;              % semi-minor axis [m]
c = 0.372;              % semi-minor axis (b = c, axisymmetric) [m]
V = 0.535;              % envelope volume [m^3]
 
%% ── Rigid-Body Inertial Properties (Table 6.2) ──────────────────────────────
m   = 0.460;            % total vehicle mass [kg]
Ix  = 0.04657;          % moment of inertia about x-axis [kg·m^2]
Iy  = 0.0737;           % moment of inertia about y-axis [kg·m^2]
Iz  = 0.0873;           % moment of inertia about z-axis [kg·m^2]
xg  = 0.0;              % CG offset along x from CB [m]
yg  = 0.0;              % CG offset along y from CB [m]
zg  = 0.178;            % CG offset along z from CB (gondola weight) [m]
 
%% ── Added (Virtual) Mass Coefficients (Table 6.3) ───────────────────────────
% Sign convention: diagonal of MA = -[Xu_dot, Yv_dot, Zw_dot, Kp_dot, Mq_dot, Nr_dot]
Xu_dot = -0.1091;       % [kg]
Yv_dot = -0.3120;       % [kg]
Zw_dot = -0.3120;       % [kg]
Kp_dot =  0.0000;       % [kg·m^2]
Mq_dot = -0.0197;       % [kg·m^2]
Nr_dot = -0.0197;       % [kg·m^2]
 
%% ── Linear Damping Coefficients (Table 6.7) ─────────────────────────────────
% Low-speed indoor flight: quadratic terms neglected (see §5.2.3)
Xu = 0.1900;            % [kg/s]
Yv = 0.7520;            % [kg/s]
Zw = 0.3366;            % [kg/s]
Kp = 5.4265;            % [kg·m^2/s]
Mq = 6.2059;            % [kg·m^2/s]
Nr = 0.2450;            % [kg·m^2/s]
 
%% ── Actuator Moment Arms (Table 6.2) ─────────────────────────────────────────
% Each column: [lx; ly; lz] for that actuator
% Motor 1: left lateral (Park 250)   Motor 2: right lateral (Park 180)   Motor 3: vertical
lx = [  0.0;   0.0;  0.0  ];   % x moment arms [m]
ly = [ -0.461;  0.461;  0.0  ]; % y moment arms [m]
lz = [ -0.175; -0.175; 0.178]; % z moment arms [m]
 
%% ── Thrust Coefficients (§6.3.1) ────────────────────────────────────────────
% Linear fit: T [N] = k * u  where u is PWM (brushless) or voltage (DC)
% For simulation, inputs are in Newtons directly (K is applied externally).
k_motor1 = 0.001538;    % Park 250: slope [N/PWM]
k_motor2 = 0.000610;    % Park 180: slope [N/PWM]
k_motor3 = 0.001538;    % vertical motor (assumed Park 250): [N/PWM]
 
%% ── Derived Quantities ───────────────────────────────────────────────────────
W = m * g_acc;          % vehicle weight [N]
B = rho * V * g_acc;    % buoyancy force [N]
net_lift = B - W;       % net upward force at hover [N] (positive → vehicle rises)
 
fprintf('═══════════════════════════════════════\n');
fprintf('  Airship Parameter Summary\n');
fprintf('═══════════════════════════════════════\n');
fprintf('  Weight W        = %.4f N\n', W);
fprintf('  Buoyancy B      = %.4f N\n', B);
fprintf('  Net lift (B-W)  = %.4f N  (vertical motor must provide %.4f N downward)\n', net_lift, net_lift);
fprintf('  Gondola offset  = %.3f m below CB\n', zg);
fprintf('═══════════════════════════════════════\n\n');
 
%% ── Full Mass Matrix M (eq. 5.10) ────────────────────────────────────────────
MRB = [  m,      0,      0,      0,      m*zg,   0;
          0,      m,      0,    -m*zg,     0,      0;
          0,      0,      m,      0,       0,      0;
          0,    -m*zg,    0,      Ix,      0,      0;
         m*zg,   0,       0,      0,       Iy,     0;
          0,      0,      0,      0,       0,      Iz ];
 
MA = -diag([Xu_dot, Yv_dot, Zw_dot, Kp_dot, Mq_dot, Nr_dot]);
 
M_full = MRB + MA;
 
%% ── Linear Damping Matrix D₀ (eq. 5.15) ─────────────────────────────────────
D0 = -diag([Xu, Yv, Zw, Kp, Mq, Nr]);  % negative: damping opposes motion
 
%% ── Thrust Configuration Matrix T (eq. 5.21–5.22) ───────────────────────────
% Motor orientations assumed from gondola description:
%   Motors 1 & 2 (lateral): thrust in +x (forward) body axis
%   Motor 3 (vertical):      thrust in +z (downward) body axis, counteracts buoyancy
%
% τ_i = [Fx_i; Fy_i; Fz_i; ly_i*Fz_i - lz_i*Fy_i; lz_i*Fx_i - lx_i*Fz_i; lx_i*Fy_i - ly_i*Fx_i]
%
% Motor 1 thrust direction: +x
T_col1 = [1;               % Fx
           0;               % Fy
           0;               % Fz
           ly(1)*0-lz(1)*0; % roll moment:  ly*Fz - lz*Fy = 0
           lz(1)*1-lx(1)*0; % pitch moment: lz*Fx - lx*Fz = lz(1)
           lx(1)*0-ly(1)*1];% yaw moment:   lx*Fy - ly*Fx = -ly(1)
 
% Motor 2 thrust direction: +x
T_col2 = [1;
           0;
           0;
           ly(2)*0-lz(2)*0;
           lz(2)*1-lx(2)*0;
           lx(2)*0-ly(2)*1];
 
% Motor 3 thrust direction: +z (downward, opposing net buoyancy)
T_col3 = [0;
           0;
           1;
           ly(3)*1-lz(3)*0;  % = 0
           lz(3)*0-lx(3)*1;  % = 0
           lx(3)*0-ly(3)*1]; % = 0
 
T = [T_col1, T_col2, T_col3];   % 6×3 thrust configuration matrix
 
K_thrust = diag([k_motor1, k_motor2, k_motor3]);  % 3×3 thrust coefficient matrix
 
fprintf('Thrust configuration matrix T (6×3):\n');
disp(T);
fprintf('Thrust coefficient matrix K (diagonal, N/PWM):\n');
disp(diag(K_thrust)');
