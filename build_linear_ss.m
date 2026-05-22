%% 
%% build_linear_ss.m
% Constructs the linearized state-space model (A, B, C, D) of the airship
% around the hover equilibrium, following the Jacobian linearization
% derived from Sponaugle (2025) MSME thesis, Chapter 5.
%
% State vector ξ ∈ ℝ¹² (order matches thesis notation):
%   ξ = [x, y, z, φ, θ, ψ,  u, v, w, p, q, r]ᵀ
%         ─── η (inertial pose) ───  ── ν (body vel) ──
%
% Inputs:  u_cmd ∈ ℝ³  — actuator force commands [N]
%          (u1 = left lateral, u2 = right lateral, u3 = vertical)
%
% Outputs: All 12 states (full state vector) by default.
%          Modify C_mat below for specific sensor outputs.
%
% Run airship_params.m first, or call as a section within run_simulation.m.
 
%% ── Step 1: Equilibrium Conditions ──────────────────────────────────────────
% Hover: zero body velocity, level attitude, vertical motor balances net lift.
eta0 = [0; 0; 0; 0; 0; 0];     % η₀ = [x₀, y₀, z₀, 0, 0, 0]ᵀ
nu0  = zeros(6, 1);             % ν₀ = 0 (all velocities zero)
 
% Equilibrium thrust: vertical motor must push DOWN by net_lift [N]
% Lateral motors are idle at hover (no heading error).
u0 = [0; 0; net_lift];          % [N] — u3 counteracts buoyancy excess
 
fprintf('Equilibrium thrust u₀ = [%.4f, %.4f, %.4f] N\n\n', u0(1), u0(2), u0(3));
 
%% ── Step 2: Kinematic Sub-Block of A (top 6 rows) ───────────────────────────
% η̇ = J(θ)·ν
% At hover: φ₀=0, θ₀=0 → J(θ₀) = I₆
% ∂η̇/∂η = ∂J/∂η·ν₀ = 0  (ν₀ = 0)
% ∂η̇/∂ν = J(θ₀)     = I₆
 
J0 = eye(6);   % rotation/transformation at level trim
 
A_kinem_eta = zeros(6, 6);  % ∂η̇/∂η
A_kinem_nu  = J0;            % ∂η̇/∂ν
 
%% ── Step 3: Restoring Force Jacobian ∂g/∂η (at φ₀=θ₀=0) ───────────────────
% g(η) from eq. 5.19 (simplified with xg=yg=0, zB=0):
%   g₁ = (W-B)sin(θ)
%   g₂ = -(W-B)cos(θ)sin(φ)
%   g₃ = -(W-B)cos(θ)cos(φ)
%   g₄ = zg·W·cos(θ)·sin(φ)
%   g₅ = zg·W·sin(θ)
%   g₆ = 0
%
% Partial derivatives w.r.t. [x, y, z, φ, θ, ψ] at φ=θ=0:
 
dg_deta = zeros(6, 6);
 
%  ∂g₁/∂θ = (W-B)cos(θ)|₀ = (W-B)
dg_deta(1, 5) = (W - B);
 
%  ∂g₂/∂φ = -(W-B)cos(θ)cos(φ)|₀ = -(W-B)
dg_deta(2, 4) = -(W - B);
 
%  ∂g₃/∂φ = (W-B)cos(θ)sin(φ)|₀ = 0  → no entry needed
 
%  ∂g₄/∂φ = zg·W·cos(θ)·cos(φ)|₀ = zg·W   (gravitational restoring in roll)
dg_deta(4, 4) = zg * W;
 
%  ∂g₅/∂θ = zg·W·cos(θ)|₀ = zg·W            (gravitational restoring in pitch)
dg_deta(5, 5) = zg * W;
 
%% ── Step 4: Kinetic Sub-Block of A (bottom 6 rows) ──────────────────────────
% ν̇ = M⁻¹[τ_b - C(ν)ν - D(ν)ν - g(η)]
% At ν₀ = 0: C(0) = 0 (all Coriolis terms are quadratic in velocity)
%            ∂C(ν)ν/∂ν|₀ = 0 (for the same reason)
%            D(ν)ν|₀ → linear: ∂/∂ν = D₀  (linear drag, eq. 5.15)
%
% ∂ν̇/∂η = -M⁻¹ · ∂g/∂η
% ∂ν̇/∂ν = -M⁻¹ · D₀
 
M_inv = inv(M_full);
 
A_kinet_eta = -M_inv * dg_deta;    % ∂ν̇/∂η
A_kinet_nu  = -M_inv * (-D0);      % ∂ν̇/∂ν = -M⁻¹ · D₀
% Note: D₀ as defined in params is -diag(...), so -D₀ = diag(Xu,...,Nr) > 0
% Correct: ∂ν̇/∂ν = M⁻¹ · D₀ where D₀ = -diag(damp coeff)
 
%% ── Assemble A (12×12) ───────────────────────────────────────────────────────
A_mat = [ A_kinem_eta,  A_kinem_nu;
          A_kinet_eta,  A_kinet_nu ];
 
%% ── Step 5: Input Matrix B (12×3) ───────────────────────────────────────────
% Control enters only through kinetic equation:
%   ∂ν̇/∂u_cmd = M⁻¹ · T
%
% Inputs are forces in Newtons (T maps 3 actuator forces → 6-DOF wrench).
% To use raw PWM or voltage instead, replace T with T·K_thrust.
 
B_mat = [ zeros(6, 3);
          M_inv * T      ];   % 12×3 — force [N] → state derivative
 
% If you prefer raw PWM inputs (u in PWM units), uncomment:
% B_mat_pwm = [ zeros(6,3); M_inv * T * K_thrust ];
 
%% ── Step 6: Output Matrix C, D ──────────────────────────────────────────────
% Full state output (for LQR, observer design, or Vicon ground truth).
% Uncomment the block below for position+attitude only (6 outputs).
 
C_mat = eye(12);             % 12 outputs: full state
D_mat = zeros(12, 3);        % no direct feedthrough
 
% ── Position + attitude only (uncomment if desired) ──
% C_mat = [eye(6), zeros(6)];   % 6 outputs: [x,y,z,φ,θ,ψ]
% D_mat = zeros(6, 3);
 
%% ── Step 7: Build MATLAB ss object ──────────────────────────────────────────
state_names = {'x','y','z','phi','theta','psi','u_vel','v_vel','w_vel','p','q','r'};
input_names = {'F1_N','F2_N','F3_N'};
output_names = state_names(1:size(C_mat,1));
 
sys_linear = ss(A_mat, B_mat, C_mat, D_mat, ...
                'StateName',  state_names, ...
                'InputName',  input_names, ...
                'OutputName', output_names);
 
%% ── Step 8: Quick Diagnostics ────────────────────────────────────────────────
eigs_A = eig(A_mat);
unstable = sum(real(eigs_A) > 1e-6);
fprintf('────────────────────────────────────────\n');
fprintf('  Linear Model Diagnostics\n');
fprintf('────────────────────────────────────────\n');
fprintf('  System order          : %d states, %d inputs, %d outputs\n', ...
        size(A_mat,1), size(B_mat,2), size(C_mat,1));
[isCtrb, nCtrb] = deal(rank(ctrb(A_mat, B_mat)));
[isObsv, nObsv] = deal(rank(obsv(A_mat, C_mat)));
fprintf('  Controllability rank  : %d / %d  (%s)\n', nCtrb, 12, ...
        iif(nCtrb==12,'FULL','RANK DEFICIENT — check actuator config'));
fprintf('  Observability rank    : %d / %d  (%s)\n', nObsv, 12, ...
        iif(nObsv==12,'FULL','RANK DEFICIENT — check sensor config'));
fprintf('  Unstable open-loop modes: %d\n', unstable);
fprintf('  Eigenvalues of A:\n');
fprintf('    %+.4f%+.4fi\n', [real(eigs_A), imag(eigs_A)]');
fprintf('────────────────────────────────────────\n\n');
 
fprintf('State-space model built successfully → variable: sys_linear\n');
fprintf('Matrices available: A_mat (12×12), B_mat (12×3), C_mat, D_mat\n\n');
 
%% ── Helper (inline if-else) ──────────────────────────────────────────────────
function out = iif(cond, a, b)
    if cond, out = a; else, out = b; end
end
 
