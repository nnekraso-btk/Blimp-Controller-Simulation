function 10m_CLF_CBF_RNDM_Disturbance()
%% blimpFormation — Leader-Follower Formation Control
%
% Two modes, switch with M key:
%
%  MODE 1  "AUTO LEADER"   — Leader A flies a figure-8 path autonomously.
%                            Follower B is manual (WASD). Try to keep up.
%
%  MODE 2  "AUTO FOLLOWER" — Leader A is manual (arrow keys).
%                            Follower B autonomously tracks A at follow_dist.
%
% Both modes show the full comm-zone ring system from blimpSim3D.
%
% CONTROLS:
%   Arrow keys  — Agent A (leader in Mode 2, ignored in Mode 1)
%   W/A/S/D     — Agent B (follower in Mode 1, ignored in Mode 2)
%   M           — Toggle mode
%   R           — Reset both agents
%   Q / Esc     — Quit

%% ═══════════════════════════════════════════════════════════
%% PARAMETERS
%% ═══════════════════════════════════════════════════════════
% Effective masses = rigid body + added mass (Lamb's k-factor, Table 6.3)
% Xu_dot=-0.1091, Yv_dot=-0.3120, Zw_dot=-0.3120, Nr_dot=-0.0197
P.m_surge = 0.460 + 0.1091;  % m - Xu_dot = 0.5691 kg
P.m_sway  = 0.460 + 0.3120;  % m - Yv_dot = 0.7720 kg  (higher — more added mass)
P.m_heave = 0.460 + 0.3120;  % m - Zw_dot = 0.7720 kg
P.m       = 0.460;            % rigid body mass [kg]
P.Iz      = 0.0873 + 0.0197;  % Iz - Nr_dot = 0.1070 kg*m^2
P.Xu = 0.1900;  % surge damping  [kg/s] (Table 6.7)
P.Yv = 0.7520;  % sway damping   [kg/s] — 4x Xu: strong lateral resistance
P.Zw = 0.3366;  % heave damping  [kg/s]
P.Nr = 0.2450;  % yaw damping    [kg*m^2/s]
P.ly = 0.461;
P.F_max  = 1.07;
P.F_step = 0.40;
P.F_vert = 0.55;
P.a = 0.55;  P.b = 0.22;  P.c = 0.22;

% Comm zones
% Zone distances scaled to WVU Vicon lab (~10m x 10m capture volume).
% comm_min_red set to 3.0m to give 1.16m physical clearance at the CBF
% boundary: 3.0 - 2×0.918 = 1.16m. Previous 2.0m gave only 0.16m —
% any inward velocity at boundary meant motor could not stop before contact.
P.comm_min_red    = 3.00;   % collision danger [m] — ≥ 2×a + margin
P.comm_min_yellow = 4.00;   % inner warning [m]
P.comm_ideal_lo   = 4.00;   % ideal zone lower bound [m]
P.comm_ideal_hi   = 6.25;   % ideal zone upper bound [m]
P.comm_max_yellow = 7.50;   % outer warning [m]
P.comm_max_red    = 10.00;  % comm loss boundary [m]

% Autonomous leader path  (figure-8 lemniscate)
P.path_A     = 2.5;    % amplitude [m]
P.path_omega = 0.22;   % angular speed [rad/s]  — one loop ~28 s
P.path_z     = 1.0;    % constant cruise altitude [m]

% Autonomous follower controller gains  (proportional only for clarity)
P.follow_dist = 5.50;         % desired separation [m] — gives 4.58m from A to B's rear
P.cone_half_angle = pi/3;      % ±60° cone half-angle — h3 CBF boundary [rad]
P.cone_warn_frac  = 0.75;      % soft barrier activates at 75% of cone (45°)
P.clf_p_cone      = 40.0;      % h3 slack penalty — lower than CLF so cone
                               % yields before convergence under motor limits
P.clf_p_slack     = 80.0;      % CLF slack penalty (convergence, yields last)
% Follower controller gains — retuned for world-frame inertia model
%
% Old model had zeta=0.30 (heavily underdamped — the floatiness).
% Target: zeta=0.70 (critically damped), settle time ~2s.
% Derived: wn = sqrt(Kp/m) = 2.48 rad/s  →  Kp = wn^2 * m = 3.50
%          Kd = 2 * zeta * wn * m = 2 * 0.70 * 2.48 * 0.569 = 1.97 ≈ 2.00
% Kff reduced: S.Vx now includes disturbance drift that Kff misreads
% as 'B already moving fast — back off', which fights the recovery.
P.Kp      = 3.50;   % N/m      — saturates motor at ep > 0.31m (correct)
P.Kd      = 3.40;   % N/(m/s)  — zeta=1.2: overdamped, zero overshoot guaranteed
P.Kff     = 0.55;   % N/(m/s)  — increased: 0.92N tracking during sharp turns
P.Kp_heave= 2.00;   % N/m      — heave gains match horizontal responsiveness
P.Kp_yaw  = 4.50;   % yaw gain — autoLeader and autoFollower
P.Kd_yaw  = 0.60;   % yaw rate damping — prevents overshoot
P.Kff_yaw = 0.30;   % Coriolis feedforward on yaw — pre-compensates beta_dot
                    % so nose stays ahead of sway rather than reacting to it
% CLF-CBF QP safety filter
% CBF boundaries: use yellow-zone edges so QP pushes back before red
% CBF boundaries: use actual danger zones (red) so QP only fires
% when genuinely close to a safety violation, not during normal following
P.cbf_d_min   = P.comm_min_red;     % [m] inner boundary = 2.00 m
P.cbf_d_max   = P.comm_max_red;     % [m] outer boundary = 10.00 m
% ECBF decay rates: lower = narrower pre-warning zone = less interference
P.cbf_alpha1  = 0.35;   % inner ECBF decay rate — further reduced to limit spiral seeding
P.cbf_alpha2  = 0.35;   % was 0.50; outward correction was still energetic enough to
                         % seed Coriolis spiral when inward disturbance applied
P.cbf_beta1   = 0.80;   % outer ECBF decay rate
P.cbf_beta2   = 0.80;
% Yaw rate soft cap: limits controller-commanded spin to prevent Coriolis
% positive feedback. At r=1.5 rad/s the Coriolis sway input m*u*r/m_sway
% can grow faster than the heading controller corrects it. Physical drag
% alone caps terminal yaw at 4.03 rad/s; this cap is a controller preference
% sitting well inside that envelope, not an artificial physics constraint.
% Surge has no separate cap — existing drag Xu*u naturally limits terminal
% surge to 1.07/0.190 = 5.63 m/s; alpha reduction prevents reaching it.
P.r_soft_cap  = 1.00;   % [rad/s] yaw rate soft cap — tightened to reduce Coriolis
                         % spiral amplification after inner CBF correction
P.vel_damp_k  = 4.00;   % damping gain on yaw rate excess
% CLF convergence rates
P.clf_gamma1  = 1.20;   % CLF first  decay rate  [1/s]
P.clf_gamma2  = 1.20;   % CLF second decay rate  [1/s]
% Passthrough thresholds aligned with yellow zone boundaries:
% QP activates exactly when the HUD turns yellow — same visual and physical warning.
% h1 at yellow inner (d=1.20m): 1.2^2 - 0.8^2 = 0.80
% h2 at yellow outer (d=3.00m): 4.0^2 - 3.0^2 = 7.00
% QP activation thresholds
% Inner: fire at d < 1.80m  (0.60m before yellow, 1.00m before red)
% Outer: fire at d > 2.00m  (1.00m before yellow at 3.00m)
% Outer threshold tightened because tangential escape builds velocity
% silently (h2_dot ≈ 0) — need more runway to react.
% h1 at d=1.80m: 1.8^2 - 0.8^2 = 2.60
% h2 at d=2.00m: 4.0^2 - 2.0^2 = 12.0
% h1 at d=4.50m: 4.5^2 - 3.0^2 = 11.25 (updated for new d_min=3.0m)
% h2 at d=5.00m: 10^2 - 5.0^2  = 75.00
P.cbf_h1_thresh        = 11.25;  % QP fires when d < 4.50m
P.cbf_h2_thresh        = 75.00;  % QP fires when d > 5.00m
P.cbf_broadside_thresh = 0.50;   % activate when within ~60° of broadside
P.cbf_broadside_d_min  = 5.00;   % [m] broadside check only when d > this

% No slack (delta=0): CLF is a hard constraint — no obstacles so full convergence required
% If CLF causes QP infeasibility (motor saturation), falls back to CBF-only
P.cbf_enabled = true;   % set false to disable filter and compare behaviour

% CBF boundaries moved to actual danger zones (red) not yellow warning zones.
% Using yellow zones placed the pre-warning region directly over the formation
% distance, causing the QP to fire constantly during normal following and
% produce wild yaw oscillations. Red zones give the nominal controller space.
% Disturbance parameters — radially-outward square-wave force on Agent B
% Direction is always away from A so the outer barrier is directly tested.
% The controller never sees this force — it only sees the resulting drift.
P.dist_enabled = true;   % set false to run undisturbed for comparison
P.dist_mag         = 0.40;  % simulation disturbance magnitude [N]
% +1 = radially outward (away from A) — tests outer comm / h2 barrier
% -1 = radially inward  (toward A)    — tests inner comm / h1 barrier
% The CBF margin (dist_max_force) always uses the magnitude, not the sign,
% so it pre-loads the motor against the actual applied force regardless
% of which barrier is being tested.
P.dist_direction   = 0;     % +1 outward, -1 inward, 0 = randomized each cycle
% When dist_direction=0, a random unit vector is drawn once per ON cycle
% and held constant for that cycle's duration — matching a real gust that
% has a fixed direction during its event rather than changing each tick.
% Robust CBF margin: worst-case disturbance the motor must always be ready
% to fight, regardless of direction. Set equal to dist_mag so the guarantee
% holds for the largest disturbance actually present in the simulation.
% Cannot exceed motor authority (1.07N) — doing so makes the QP infeasible.
% At 0.40N, disturbance consumes 37% of motor authority, leaving 63% for
% formation following. This is the correct engineering balance.
P.dist_max_force   = 0.40;  % [N] worst-case disturbance magnitude for CBF margin
% Leader speed scaling parameters
P.leader_T2        = 1.00;  % [s] prediction horizon for B's worst-case trajectory
P.leader_Kbrake    = 1.50;  % braking gain for Layer 2/3 deceleration [N/(m/s)]
% Layer 3 timeout: after this many seconds in HOLD, A resumes at reduced speed.
% Without this, a working Layer 2 (A stops cleanly) makes recovery impossible —
% both agents sit still with no exit. 10s gives B time to navigate to last-A
% before A resumes, reducing the chance of them missing each other.
P.layer3_timeout      = 10.0; % [s] HOLD duration before resuming at reduced speed
P.layer3_resume_scale = 0.30; % fraction of normal speed on timeout resume
% Hysteresis margin for L3 exit: B must return this far inside the outer
% boundary before the comm_lost flag clears and the timer resets.
% Without this, a ±0.01m oscillation at exactly d_max toggles comm_lost
% every tick, preventing the timer from ever reaching layer3_timeout.
P.layer3_reentry_m    = 1.50; % [m] must come this far inside d_max to exit L3
P.dist_t_on    = 5.0;    % seconds the disturbance is active each cycle
P.dist_t_off   = 4.0;    % seconds of calm between bursts (recovery window)
P.dist_t_start = 8.0;    % seconds before first disturbance — let formation settle first

%% ═══════════════════════════════════════════════════════════
%% ELLIPSOID MESH
%% ═══════════════════════════════════════════════════════════
N = 20;
[us,vs] = meshgrid(linspace(0,2*pi,N), linspace(0,pi,N));
E0.x = P.a.*cos(us).*sin(vs);
E0.y = P.b.*sin(us).*sin(vs);
E0.z = P.c.*cos(vs);
E0.n = N;

%% ═══════════════════════════════════════════════════════════
%% COLOURS
%% ═══════════════════════════════════════════════════════════
BG  = [0.04 0.08 0.05];
BG2 = [0.02 0.05 0.03];
GRN = [0.17 1.00 0.56];
DGR = [0.10 0.40 0.22];
CYN = [0.10 0.85 0.95];
CYN2= [0.05 0.55 0.70];

%% ═══════════════════════════════════════════════════════════
%% FIGURE + AXES
%% ═══════════════════════════════════════════════════════════
fig = figure('Name','WVU Airship — Formation Control', ...
    'NumberTitle','off','Color',BG,'Position',[40 40 1280 720], ...
    'KeyPressFcn',@onKeyDown,'KeyReleaseFcn',@onKeyUp, ...
    'CloseRequestFcn',@onClose,'Toolbar','none','Menubar','none');

    function ax = ax2D(pos,xl,yl,ttl)
        ax = axes('Parent',fig,'Units','pixels','Position',pos, ...
            'Color',BG2,'XColor',DGR,'YColor',DGR, ...
            'GridColor',[0.07 0.20 0.11],'GridAlpha',1, ...
            'XGrid','on','YGrid','on','Box','on', ...
            'FontName','Courier New','FontSize',8);
        hold(ax,'on'); axis(ax,'manual');
        title(ax,ttl,'Color',GRN,'FontName','Courier New','FontSize',9);
        xlabel(ax,xl,'Color',DGR); ylabel(ax,yl,'Color',DGR);
    end

axS = ax2D([18  375 335 310],'X [m]','Z [m]','SIDE  (X - Z)');
xlim(axS,[-15 15]); ylim(axS,[-1 3]);

axT = ax2D([365 375 335 310],'X [m]','Y [m]','TOP  (X - Y)');
xlim(axT,[-15 15]); ylim(axT,[-15 15]);

axF = ax2D([18   45 335 300],'Y [m]','Z [m]','FRONT CAMERA  (Y - Z)');
xlim(axF,[-15 15]); ylim(axF,[-1 3]);

ax3 = axes('Parent',fig,'Units','pixels','Position',[365 45 500 300], ...
    'Color',[0.02 0.04 0.03],'XColor',DGR,'YColor',DGR,'ZColor',DGR, ...
    'GridColor',[0.07 0.20 0.11],'GridAlpha',0.8, ...
    'XGrid','on','YGrid','on','ZGrid','on','Box','on', ...
    'FontName','Courier New','FontSize',7,'Projection','perspective');
hold(ax3,'on'); view(ax3,38,22);
xlim(ax3,[-15 15]); ylim(ax3,[-15 15]); zlim(ax3,[-1 3]);
title(ax3,'3D PERSPECTIVE','Color',GRN,'FontName','Courier New','FontSize',9);
xlabel(ax3,'X','Color',DGR); ylabel(ax3,'Y','Color',DGR); zlabel(ax3,'Z','Color',DGR);

%% ── Right panel ───────────────────────────────────────────────────────────
axP = axes('Parent',fig,'Units','pixels','Position',[878 45 390 650], ...
    'Color',BG2,'Visible','off');
hold(axP,'on'); xlim(axP,[0 1]); ylim(axP,[0 1]);

% Mode banner (top of panel)
svModeBanner = text(axP,0.5,0.97,'MODE 2','Color',GRN,'FontName','Courier New', ...
    'FontSize',11,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');
svModeDesc = text(axP,0.5,0.92,'AUTO FOLLOWER','Color',CYN,'FontName','Courier New', ...
    'FontSize',8,'HorizontalAlignment','center','Units','normalized');
svModeHint = text(axP,0.5,0.875,'Arrows=Leader  B=auto','Color',DGR,'FontName','Courier New', ...
    'FontSize',7,'HorizontalAlignment','center','Units','normalized');

% Comm link section
text(axP,0.5,0.84,'COMM LINK','Color',[0.5 0.5 0.3],'FontName','Courier New', ...
    'FontSize',7,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');
svABDist = text(axP,0.5,0.79,'-.-- m','Color',[0.3 0.9 0.3],'FontName','Courier New', ...
    'FontSize',13,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');
svCommZone = text(axP,0.5,0.745,'IDEAL','Color',[0.3 0.9 0.3],'FontName','Courier New', ...
    'FontSize',8,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');

% Zone bar
zone_y0=0.695; zone_h=0.03; zone_w=0.18;
zClrs = {[0.6 0.1 0.1],[0.6 0.5 0.05],[0.1 0.5 0.15],[0.6 0.5 0.05],[0.6 0.1 0.1]};
for zi=1:5
    patch(axP,[0.05+(zi-1)*zone_w 0.05+zi*zone_w 0.05+zi*zone_w 0.05+(zi-1)*zone_w], ...
        [zone_y0 zone_y0 zone_y0+zone_h zone_y0+zone_h], ...
        zClrs{zi},'EdgeColor',[0.02 0.05 0.03],'LineWidth',0.5);
end
for zi=1:5
    zl={'CRIT','WARN','OK','WARN','CRIT'};
    text(axP,0.05+(zi-0.5)*zone_w,zone_y0+zone_h/2+0.005,zl{zi}, ...
        'Color',[0.8 0.8 0.8],'FontName','Courier New','FontSize',5, ...
        'HorizontalAlignment','center','Units','normalized');
end
svZonePtr = plot(axP,0.5,zone_y0-0.012,'^','Color',[0.3 0.9 0.3], ...
    'MarkerFaceColor',[0.3 0.9 0.3],'MarkerSize',6);

% Leader/follower state labels
text(axP,0.175,0.645,'LEADER (A)','Color',GRN,'FontName','Courier New', ...
    'FontSize',6,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');
text(axP,0.49,0.645,'FOLLOWER (B)','Color',CYN,'FontName','Courier New', ...
    'FontSize',6,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');
text(axP,0.80,0.645,'SYSTEM','Color',[0.8 0.7 0.3],'FontName','Courier New', ...
    'FontSize',6,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');

sNm={'x','y','z','psi','u','w','r'};
sUn={'m','m','m','deg','m/s','m/s','r/s'};
svA=gobjects(7,1); svB=gobjects(7,1);
for i=1:7
    yp=0.59-(i-1)*0.060;
    text(axP,0.03,yp+0.025,sprintf('%s[%s]',upper(sNm{i}),sUn{i}), ...
        'Color',DGR,'FontName','Courier New','FontSize',5,'Units','normalized');
    svA(i)=text(axP,0.03,yp,'0.00','Color',GRN,'FontName','Courier New', ...
        'FontSize',7,'FontWeight','bold','Units','normalized');
    svB(i)=text(axP,0.36,yp,'0.00','Color',CYN,'FontName','Courier New', ...
        'FontSize',7,'FontWeight','bold','Units','normalized');
end

% Controller error readout (Mode 2) — moved to bottom
text(axP,0.25,0.05,'CTRL ERROR','Color',DGR,'FontName','Courier New', ...
    'FontSize',6,'HorizontalAlignment','center','Units','normalized');
svCtrlErr = text(axP,0.25,0.015,'dx -- dy -- dz --','Color',[0.7 0.7 0.5], ...
    'FontName','Courier New','FontSize',6,'HorizontalAlignment','center','Units','normalized');

% ── System status column (right third) ──────────────────────────────────
% Labels
sysY0 = 0.590;  sysX = 0.67;  dY = 0.060;
text(axP,sysX,sysY0+0.025,'LAYER',     'Color',DGR,'FontName','Courier New','FontSize',5,'Units','normalized');
text(axP,sysX,sysY0-dY+0.025,'SPD SCALE','Color',DGR,'FontName','Courier New','FontSize',5,'Units','normalized');
text(axP,sysX,sysY0-2*dY+0.025,'PRED d','Color',DGR,'FontName','Courier New','FontSize',5,'Units','normalized');
text(axP,sysX,sysY0-3*dY+0.025,'h1 (inn)','Color',DGR,'FontName','Courier New','FontSize',5,'Units','normalized');
text(axP,sysX,sysY0-4*dY+0.025,'h2 (out)','Color',DGR,'FontName','Courier New','FontSize',5,'Units','normalized');
text(axP,sysX,sysY0-5*dY+0.025,'CBF','Color',DGR,'FontName','Courier New','FontSize',5,'Units','normalized');
text(axP,sysX,sysY0-6*dY+0.025,'DIST','Color',DGR,'FontName','Courier New','FontSize',5,'Units','normalized');
% Values (updated each frame)
svLayer   =text(axP,sysX,sysY0,'L1','Color',[0.3 0.9 0.3],'FontName','Courier New','FontSize',7,'FontWeight','bold','Units','normalized');
svSpdScl  =text(axP,sysX,sysY0-dY,'1.00','Color',[0.3 0.9 0.3],'FontName','Courier New','FontSize',7,'FontWeight','bold','Units','normalized');
svPredD   =text(axP,sysX,sysY0-2*dY,'-.--','Color',[0.6 0.6 0.4],'FontName','Courier New','FontSize',7,'FontWeight','bold','Units','normalized');
svH1      =text(axP,sysX,sysY0-3*dY,'--','Color',[0.6 0.6 0.4],'FontName','Courier New','FontSize',7,'FontWeight','bold','Units','normalized');
svH2      =text(axP,sysX,sysY0-4*dY,'--','Color',[0.6 0.6 0.4],'FontName','Courier New','FontSize',7,'FontWeight','bold','Units','normalized');
svCBFsys  =text(axP,sysX,sysY0-5*dY,'--','Color',[0.6 0.6 0.4],'FontName','Courier New','FontSize',7,'FontWeight','bold','Units','normalized');
svDistSys =text(axP,sysX,sysY0-6*dY,'OFF','Color',DGR,'FontName','Courier New','FontSize',7,'FontWeight','bold','Units','normalized');

% Instruction bar
annotation(fig,'textbox',[0 0 1 0.04], ...
    'String','  Arrows=Leader(M2)/ignored(M1)   WASD=Follower(M1)/ignored(M2)   M=Mode   R=Reset   Q=Quit', ...
    'Color',[0.2 0.6 0.3],'BackgroundColor',[0.02 0.06 0.03], ...
    'EdgeColor',[0.07 0.25 0.12],'FontName','Courier New','FontSize',8, ...
    'VerticalAlignment','middle');

%% ═══════════════════════════════════════════════════════════
%% GRAPHIC OBJECTS
%% ═══════════════════════════════════════════════════════════
% Agent A (green) ─────────────────────────────────────────────
hA_TrailS= plot(axS,NaN,NaN,'-','Color',GRN*0.5,'LineWidth',1);
hA_EnvS  = patch(axS,'XData',0,'YData',0,'FaceColor',[0.05 0.22 0.10], ...
    'EdgeColor',GRN,'LineWidth',1.2);
hA_GndS  = patch(axS,'XData',0,'YData',0,'FaceColor',[0.03 0.12 0.06], ...
    'EdgeColor',[0.1 0.35 0.18],'LineWidth',0.7);
hA_DirS  = quiver(axS,0,0,0.5,0,0,'Color',GRN,'LineWidth',2,'MaxHeadSize',0.8,'AutoScale','off');
plot(axS,[-16 16],[-0.5 -0.5],'--','Color',[0.45 0.12 0.12],'LineWidth',0.8);
text(axS,-4.7,-0.65,'GROUND','Color',[0.45 0.12 0.12],'FontName','Courier New','FontSize',7);

hA_TrailT= plot(axT,NaN,NaN,'-','Color',GRN*0.4,'LineWidth',1);
hA_EnvT  = patch(axT,'XData',0,'YData',0,'FaceColor',[0.05 0.22 0.10], ...
    'EdgeColor',GRN,'LineWidth',1.2);
hA_DirT  = quiver(axT,0,0,0.5,0,0,'Color',GRN,'LineWidth',2,'MaxHeadSize',0.8,'AutoScale','off');
hA_LblT  = text(axT,0,0.4,'A','Color',GRN,'FontName','Courier New', ...
    'FontSize',9,'FontWeight','bold','HorizontalAlignment','center');

hA_TrailF= plot(axF,NaN,NaN,'-','Color',GRN*0.4,'LineWidth',1);
hA_EnvF  = patch(axF,'XData',0,'YData',0,'FaceColor',[0.05 0.22 0.10], ...
    'EdgeColor',GRN,'LineWidth',1.2,'FaceAlpha',0.7);
plot(axF,[-16 16],[0 0],'--','Color',[0.45 0.12 0.12],'LineWidth',0.8);
text(axF,-3.8,-0.15,'HORIZON','Color',[0.45 0.12 0.12],'FontName','Courier New','FontSize',7);

hA_Surf  = surf(ax3,E0.x,E0.y,E0.z,'FaceColor',[0.07 0.28 0.12], ...
    'EdgeColor',GRN,'FaceAlpha',0.82,'EdgeAlpha',0.2,'LineWidth',0.3);
hA_Gond3 = patch(ax3,'Vertices',zeros(8,3),'Faces',ones(6,4), ...
    'FaceColor',[0.04 0.14 0.07],'EdgeColor',[0.15 0.5 0.25],'FaceAlpha',0.9);
hA_Fin3L = patch(ax3,'XData',0,'YData',0,'ZData',0,'FaceColor',[0.06 0.20 0.10], ...
    'EdgeColor',GRN,'EdgeAlpha',0.5,'FaceAlpha',0.75);
hA_Fin3R = patch(ax3,'XData',0,'YData',0,'ZData',0,'FaceColor',[0.06 0.20 0.10], ...
    'EdgeColor',GRN,'EdgeAlpha',0.5,'FaceAlpha',0.75);
hA_Fin3T = patch(ax3,'XData',0,'YData',0,'ZData',0,'FaceColor',[0.06 0.20 0.10], ...
    'EdgeColor',GRN,'EdgeAlpha',0.5,'FaceAlpha',0.75);
hA_Dir3  = quiver3(ax3,0,0,0,0.65,0,0,0,'Color',GRN,'LineWidth',2,'MaxHeadSize',0.5,'AutoScale','off');
hA_Trail3= plot3(ax3,NaN,NaN,NaN,'-','Color',GRN*0.5,'LineWidth',1.1);

% Autonomous path preview (Mode 1 only — faint lemniscate)
t_prev = linspace(0,2*pi/P.path_omega,200);
x_prev = P.path_A.*cos(P.path_omega.*t_prev);
y_prev = P.path_A.*sin(2*P.path_omega.*t_prev)./2;
hPathPrev = plot(axT,x_prev,y_prev,'--','Color',[GRN*0.35],'LineWidth',0.8);
hPathPrev3= plot3(ax3,x_prev,y_prev,P.path_z*ones(size(t_prev)),'--','Color',GRN*0.25,'LineWidth',0.7);

% Waypoint marker (current path target)
hWpt_T   = plot(axT,NaN,NaN,'o','Color',GRN,'MarkerSize',6,'LineWidth',1.5);
hWpt_3   = plot3(ax3,NaN,NaN,NaN,'o','Color',GRN,'MarkerSize',6,'LineWidth',1.5);

% Agent B (cyan) ──────────────────────────────────────────────
hB_TrailS= plot(axS,NaN,NaN,'-','Color',CYN2,'LineWidth',1);
hB_EnvS  = patch(axS,'XData',0,'YData',0,'FaceColor',[0.04 0.22 0.30], ...
    'EdgeColor',CYN,'LineWidth',1.2);
hB_GndS  = patch(axS,'XData',0,'YData',0,'FaceColor',[0.03 0.13 0.20], ...
    'EdgeColor',CYN2,'LineWidth',0.7);
hB_DirS  = quiver(axS,0,0,0.5,0,0,'Color',CYN,'LineWidth',1.6,'MaxHeadSize',0.8,'AutoScale','off');

hB_TrailT= plot(axT,NaN,NaN,'-','Color',CYN2,'LineWidth',1);
hB_EnvT  = patch(axT,'XData',0,'YData',0,'FaceColor',[0.04 0.22 0.30], ...
    'EdgeColor',CYN,'LineWidth',1.2);
hB_DirT  = quiver(axT,0,0,0.5,0,0,'Color',CYN,'LineWidth',1.6,'MaxHeadSize',0.8,'AutoScale','off');
hB_LblT  = text(axT,0,2.4,'B','Color',CYN,'FontName','Courier New', ...
    'FontSize',9,'FontWeight','bold','HorizontalAlignment','center');

hB_TrailF= plot(axF,NaN,NaN,'-','Color',CYN2,'LineWidth',1);
hB_EnvF  = patch(axF,'XData',0,'YData',0,'FaceColor',[0.04 0.22 0.30], ...
    'EdgeColor',CYN,'LineWidth',1.2,'FaceAlpha',0.65);

hB_Surf  = surf(ax3,E0.x,E0.y,E0.z+2,'FaceColor',[0.05 0.24 0.34], ...
    'EdgeColor',CYN,'FaceAlpha',0.78,'EdgeAlpha',0.2,'LineWidth',0.3);
hB_Gond3 = patch(ax3,'Vertices',zeros(8,3),'Faces',ones(6,4), ...
    'FaceColor',[0.03 0.13 0.20],'EdgeColor',CYN2,'FaceAlpha',0.9);
hB_Fin3L = patch(ax3,'XData',0,'YData',0,'ZData',0,'FaceColor',[0.04 0.18 0.26], ...
    'EdgeColor',CYN,'EdgeAlpha',0.5,'FaceAlpha',0.75);
hB_Fin3R = patch(ax3,'XData',0,'YData',0,'ZData',0,'FaceColor',[0.04 0.18 0.26], ...
    'EdgeColor',CYN,'EdgeAlpha',0.5,'FaceAlpha',0.75);
hB_Fin3T = patch(ax3,'XData',0,'YData',0,'ZData',0,'FaceColor',[0.04 0.18 0.26], ...
    'EdgeColor',CYN,'EdgeAlpha',0.5,'FaceAlpha',0.75);
hB_Dir3  = quiver3(ax3,0,2,0,0.65,0,0,0,'Color',CYN,'LineWidth',1.8,'MaxHeadSize',0.5,'AutoScale','off');
hB_Trail3= plot3(ax3,NaN,NaN,NaN,'-','Color',CYN2,'LineWidth',1.0);

% Desired formation position marker (where follower is trying to reach)
hDesPos_T = plot(axT,NaN,NaN,'x','Color',CYN,'MarkerSize',10,'LineWidth',2);
hDesPos_3 = plot3(ax3,NaN,NaN,NaN,'x','Color',CYN,'MarkerSize',10,'LineWidth',2);
hDesLine_T= plot(axT,NaN,NaN,':','Color',CYN*0.6,'LineWidth',0.9);

% Comm zone rings (centred on A, top view + 3D)
th_c = linspace(0,2*pi,80);
hCommInner = plot(axT,NaN,NaN,'-','Color',[1 0.3 0.3],'LineWidth',1.2);
hCommLo    = plot(axT,NaN,NaN,'--','Color',[1 0.85 0.1],'LineWidth',1.0);
hCommHi    = plot(axT,NaN,NaN,'--','Color',[1 0.85 0.1],'LineWidth',1.0);
hCommOuter = plot(axT,NaN,NaN,'-','Color',[1 0.3 0.3],'LineWidth',1.2);
hCommOuter3= plot3(ax3,NaN,NaN,NaN,'-','Color',[1 0.3 0.3],'LineWidth',1.0);
hCommHi3   = plot3(ax3,NaN,NaN,NaN,'--','Color',[1 0.85 0.1],'LineWidth',0.8);

% Comm link line
hLink_T = plot(axT,NaN,NaN,'-','Color',GRN,'LineWidth',1.8);
hLink_3 = plot3(ax3,NaN,NaN,NaN,'-','Color',GRN,'LineWidth',1.8);

% Lighting
light(ax3,'Position',[2 -2 3],'Style','infinite');
light(ax3,'Position',[-1 1 -1],'Style','infinite','Color',[0.04 0.12 0.07]);
light(ax3,'Position',[0 3 2],'Style','infinite','Color',[0.04 0.16 0.22]);
lighting(ax3,'gouraud'); material(ax3,'dull');

% Floor grid
[gxf,gyf]=meshgrid(-5:1:5,-5:1:5);
surf(ax3,gxf,gyf,-ones(size(gxf)),'FaceColor','none', ...
    'EdgeColor',[0.06 0.16 0.10],'EdgeAlpha',0.5,'LineWidth',0.4);
plot(axT,0,0,'+','Color',DGR,'MarkerSize',10,'LineWidth',1.5);
% Disturbance arrow — shows direction and magnitude of current force on B
hDistArrow = quiver(axT,0,0,0,0,0,'Color',[1 0.4 0.1],'LineWidth',2.5,...
    'MaxHeadSize',0.6,'AutoScale','off');
hDistLabel = text(axT,0,0,'','Color',[1 0.4 0.1],'FontName','Courier New',...
    'FontSize',8,'FontWeight','bold');

% ── Formation cone visualisation (top view) ────────────────────────────────
% Filled wedge showing the ±60° follow cone centred on A.
% Two boundary lines for the cone edges; filled patch for the interior.
% Colour shifts green→yellow→red as B approaches the boundary.
th_cone = linspace(0, 1, 40);   % parametric, scaled in draw
hConeEdge1 = plot(axT, NaN, NaN, '--', 'Color', [0.3 0.8 0.3], 'LineWidth', 1.2);
hConeEdge2 = plot(axT, NaN, NaN, '--', 'Color', [0.3 0.8 0.3], 'LineWidth', 1.2);
hConeFill  = patch(axT, 'XData', zeros(1,3), 'YData', zeros(1,3), ...
    'FaceColor', [0.1 0.4 0.15], 'EdgeColor', 'none', 'FaceAlpha', 0.18);
hConeArc   = plot(axT, NaN, NaN, '-', 'Color', [0.3 0.8 0.3], 'LineWidth', 2.0);
% B position marker on the arc (shows exactly where B sits angularly)
hConeBMark = plot(axT, NaN, NaN, 'o', 'Color', [0.1 0.85 0.95], ...
    'MarkerSize', 7, 'LineWidth', 2);

%% ═══════════════════════════════════════════════════════════
%% BUNDLE
%% ═══════════════════════════════════════════════════════════
H.axS=axS; H.axT=axT; H.axF=axF; H.ax3=ax3; H.axP=axP;
H.hA_TrailS=hA_TrailS; H.hA_EnvS=hA_EnvS; H.hA_GndS=hA_GndS; H.hA_DirS=hA_DirS;
H.hA_TrailT=hA_TrailT; H.hA_EnvT=hA_EnvT; H.hA_DirT=hA_DirT; H.hA_LblT=hA_LblT;
H.hA_TrailF=hA_TrailF; H.hA_EnvF=hA_EnvF;
H.hA_Surf=hA_Surf; H.hA_Gond3=hA_Gond3;
H.hA_Fin3L=hA_Fin3L; H.hA_Fin3R=hA_Fin3R; H.hA_Fin3T=hA_Fin3T;
H.hA_Dir3=hA_Dir3; H.hA_Trail3=hA_Trail3;
H.hB_TrailS=hB_TrailS; H.hB_EnvS=hB_EnvS; H.hB_GndS=hB_GndS; H.hB_DirS=hB_DirS;
H.hB_TrailT=hB_TrailT; H.hB_EnvT=hB_EnvT; H.hB_DirT=hB_DirT; H.hB_LblT=hB_LblT;
H.hB_TrailF=hB_TrailF; H.hB_EnvF=hB_EnvF;
H.hB_Surf=hB_Surf; H.hB_Gond3=hB_Gond3;
H.hB_Fin3L=hB_Fin3L; H.hB_Fin3R=hB_Fin3R; H.hB_Fin3T=hB_Fin3T;
H.hB_Dir3=hB_Dir3; H.hB_Trail3=hB_Trail3;
H.hPathPrev=hPathPrev; H.hPathPrev3=hPathPrev3;
H.hWpt_T=hWpt_T; H.hWpt_3=hWpt_3;
H.hDesPos_T=hDesPos_T; H.hDesPos_3=hDesPos_3; H.hDesLine_T=hDesLine_T;
H.hCommInner=hCommInner; H.hCommLo=hCommLo; H.hCommHi=hCommHi; H.hCommOuter=hCommOuter;
H.hCommOuter3=hCommOuter3; H.hCommHi3=hCommHi3;
H.hLink_T=hLink_T; H.hLink_3=hLink_3;
H.hDistArrow=hDistArrow; H.hDistLabel=hDistLabel;
H.hConeEdge1=hConeEdge1; H.hConeEdge2=hConeEdge2;
H.hConeFill=hConeFill; H.hConeArc=hConeArc; H.hConeBMark=hConeBMark;
H.svModeBanner=svModeBanner; H.svModeDesc=svModeDesc; H.svModeHint=svModeHint;
H.svABDist=svABDist; H.svCommZone=svCommZone; H.svZonePtr=svZonePtr;
H.svA=svA; H.svB=svB; H.svCtrlErr=svCtrlErr;
H.svLayer=svLayer; H.svSpdScl=svSpdScl; H.svPredD=svPredD;
H.svH1=svH1; H.svH2=svH2; H.svCBFsys=svCBFsys; H.svDistSys=svDistSys;

%% ═══════════════════════════════════════════════════════════
%% INITIAL STATE
%% ═══════════════════════════════════════════════════════════
SA = fmState(0,    0,   0, 0);   % Agent A at origin
SB = fmState(-5.5, 0.5, 0, 0);  % Agent B near formation position

setappdata(fig,'SA',SA);
setappdata(fig,'SB',SB);
setappdata(fig,'P',P);
setappdata(fig,'H',H);
setappdata(fig,'E0',E0);
setappdata(fig,'mode',2);       % start in Mode 2 (manual leader, auto follower)
setappdata(fig,'running',true);

tmr = timer('Name','FormationTimer','Period',0.025, ...
    'ExecutionMode','fixedRate','TimerFcn',@(~,~)tick(fig));
setappdata(fig,'tmr',tmr);
start(tmr);
fprintf('blimpFormation running.  M = toggle mode  R = reset  Q = quit\n');
fprintf('Mode 2: Arrow keys = Leader A,  B follows autonomously\n');

%% ═══════════════════════════════════════════════════════════
%% NESTED CALLBACKS
%% ═══════════════════════════════════════════════════════════
    function tick(f)
        if ~ishandle(f) || ~getappdata(f,'running')
            safeStop(f); return;
        end
        SA_ = getappdata(f,'SA');
        SB_ = getappdata(f,'SB');
        P_  = getappdata(f,'P');
        H_  = getappdata(f,'H');
        E0_ = getappdata(f,'E0');
        mode_= getappdata(f,'mode');

        dt = 0.025;


        % ── Agent A: compute force command ───────────────────────────
        if mode_ == 1
            % Mode 1: autonomous leader follows figure-8
            [inpA, xwpt, ywpt] = autoLeader(SA_, P_, dt);
        else
            % Mode 2: manual leader (arrow keys)
            inpA = manualInputs(SA_, P_);
            xwpt = NaN; ywpt = NaN;
        end

        % ── Agent B: compute force command ───────────────────────────
        if mode_ == 2
            % Mode 2: autonomous follower tracks A
            [inpB, des_pos, ctrl_err] = autoFollower(SB_, SA_, P_);
        else
            % Mode 1: manual follower (WASD)
            inpB = manualInputs(SB_, P_);
            inpB.cbf_h1=NaN; inpB.cbf_h2=NaN; inpB.cbf_h3=NaN;
            inpB.cbf_active=false; inpB.clf_slack=0; inpB.qp_status='DISABLED';
            des_pos = struct('x',NaN,'y',NaN,'z',NaN);
            ctrl_err= [0 0 0];
        end

        % ── Leader speed scaling (Layers 2 & 3) ─────────────────────
        % MUST run before SA_ rk4step so scaled force reaches integrator.
        % ── Leader speed scaling (Layers 2 & 3) ──────────────────────
        dAB_now  = sqrt((SB_.x-SA_.x)^2+(SB_.y-SA_.y)^2);
        vBx_now  = SB_.u*cos(SB_.psi)-SB_.v*sin(SB_.psi);
        vBy_now  = SB_.u*sin(SB_.psi)+SB_.v*cos(SB_.psi);
        vAx_now  = SA_.u*cos(SA_.psi)-SA_.v*sin(SA_.psi);
        vAy_now  = SA_.u*sin(SA_.psi)+SA_.v*cos(SA_.psi);
        ux_AB    = (SB_.x-SA_.x)/max(dAB_now,0.01);
        uy_AB    = (SB_.y-SA_.y)/max(dAB_now,0.01);
        v_out_AB = (vBx_now-vAx_now)*ux_AB+(vBy_now-vAy_now)*uy_AB;
        a_wc     = P_.dist_max_force/P_.m_sway*(v_out_AB>=0);
        d_pred   = dAB_now+v_out_AB*P_.leader_T2+0.5*a_wc*P_.leader_T2^2;
        in_l3 = SB_.comm_lost || dAB_now > P_.cbf_d_max;
        recovered = SB_.comm_lost && ...
                    dAB_now < (P_.cbf_d_max - P_.layer3_reentry_m);
        if in_l3 && ~recovered
            % ── Layer 3 active (or hysteresis zone) ──────────────────────
            % Timer only resets when B is layer3_reentry_m inside the
            % boundary — a ±0.01m oscillation at d_max no longer resets it.
            if ~SB_.comm_lost
                SB_.t_comm_lost = SB_.t;
                SB_.comm_lost   = true;
            end
            elapsed = SB_.t - SB_.t_comm_lost;
            if elapsed >= P_.layer3_timeout
                ldr_scale = P_.layer3_resume_scale;
            else
                ldr_scale = 0.0;
            end
        else
            % ── Comms valid and clear of hysteresis band ─────────────────
            SB_.comm_lost   = false;
            SB_.t_comm_lost = 0;
            SB_.last_A_x   = SA_.x;
            SB_.last_A_y   = SA_.y;
            SB_.last_A_psi = SA_.psi;
            if d_pred >= P_.cbf_d_max
                ldr_scale = 0.0;
            elseif d_pred > P_.comm_max_yellow
                ldr_scale = max(0.0, 1-(d_pred-P_.comm_max_yellow)/...
                            (P_.cbf_d_max-P_.comm_max_yellow));
            else
                ldr_scale = 1.0;
            end
        end
        brake_Fx = -P_.leader_Kbrake*SA_.u;
        inpA.Fx  = ldr_scale*inpA.Fx+(1-ldr_scale)*brake_Fx;

        SB_.ldr_scale = ldr_scale;  % store for HUD display

        % ── Agent A integrates with correctly-scaled force ───────────
        SA_ = rk4step(SA_, inpA, P_, dt);
        % Store leader velocity so follower can estimate leader acceleration
        SA_.vAx_prev = SA_.u*cos(SA_.psi) - SA_.v*sin(SA_.psi);
        SA_.vAy_prev = SA_.u*sin(SA_.psi) + SA_.v*cos(SA_.psi);
        SA_.t     = SA_.t + dt;
        SA_.frame = SA_.frame + 1;
        if mod(SA_.frame,3)==0
            SA_.trail_x(end+1)=SA_.x; SA_.trail_y(end+1)=SA_.y; SA_.trail_z(end+1)=SA_.z;
            if length(SA_.trail_x)>400
                SA_.trail_x=SA_.trail_x(2:end); SA_.trail_y=SA_.trail_y(2:end); SA_.trail_z=SA_.trail_z(2:end);
            end
        end

        % Compute disturbance force and inject into inp before physics step.
        % The controller already ran and produced its motor commands above.
        % We add the disturbance here — after control, before integration.
        % This means the controller never anticipated it; it can only react.
        [inpB, SB_] = applyDisturbance(inpB, SB_, SA_, P_, SB_.t);

        % ── Diagnostic: print force when B settled in outer warn zone ────
        d_diag = sqrt((SB_.x-SA_.x)^2+(SB_.y-SA_.y)^2);
        if d_diag > P_.comm_max_yellow && abs(SB_.u) < 0.05 && ~SB_.dist_on
            qs = 'N/A';
            if isfield(inpB,'qp_status'), qs = inpB.qp_status; end
            fprintf('[t=%.1f] settled d=%.2fm Fx=%.3fN Mz=%.3fNm QP=%s u=%.3f\n', ...
                SB_.t, d_diag, inpB.Fx, inpB.Mz, qs, SB_.u);
        end

        SB_ = rk4step(SB_, inpB, P_, dt);
        % Store position errors so next tick can compute derivative (Kd term)
        if isfield(inpB,'ex_prev')
            SB_.ex_prev = inpB.ex_prev;
            SB_.ey_prev = inpB.ey_prev;
        end
        SB_.t     = SB_.t + dt;
        SB_.frame = SB_.frame + 1;
        if mod(SB_.frame,3)==0
            SB_.trail_x(end+1)=SB_.x; SB_.trail_y(end+1)=SB_.y; SB_.trail_z(end+1)=SB_.z;
            if length(SB_.trail_x)>400
                SB_.trail_x=SB_.trail_x(2:end); SB_.trail_y=SB_.trail_y(2:end); SB_.trail_z=SB_.trail_z(2:end);
            end
        end

        setappdata(f,'SA',SA_); setappdata(f,'SB',SB_);
        drawFormation(H_, SA_, inpA, SB_, inpB, P_, E0_, mode_, xwpt, ywpt, des_pos, ctrl_err);
        drawnow limitrate;
    end

    function onKeyDown(~,evt)
        if ~ishandle(fig), return; end
        SA_ = getappdata(fig,'SA');
        SB_ = getappdata(fig,'SB');
        mode_ = getappdata(fig,'mode');
        switch evt.Key
            % Agent A — only active when mode 2 (manual leader)
            case 'uparrow',    if mode_==2, SA_.keys.up    = 1; end
            case 'downarrow',  if mode_==2, SA_.keys.down  = 1; end
            case 'leftarrow',  if mode_==2, SA_.keys.left  = 1; end
            case 'rightarrow', if mode_==2, SA_.keys.right = 1; end
            % Agent B — only active when mode 1 (manual follower)
            case 'w', if mode_==1, SB_.keys.up    = 1; end
            case 's', if mode_==1, SB_.keys.down  = 1; end
            case 'a', if mode_==1, SB_.keys.left  = 1; end
            case 'd', if mode_==1, SB_.keys.right = 1; end
            % System
            case 'm'
                new_mode = 3 - mode_;   % toggle 1<->2
                setappdata(fig,'mode',new_mode);
                SA_.keys = struct('up',0,'down',0,'left',0,'right',0);
                SB_.keys = struct('up',0,'down',0,'left',0,'right',0);
                if new_mode==1
                    fprintf('Mode 1: Auto Leader (figure-8) + Manual Follower (WASD)\n');
                else
                    fprintf('Mode 2: Manual Leader (arrows) + Auto Follower\n');
                end
            case 'r'
                SA_ = fmState(0,   0, 0, 0);
                SB_ = fmState(0, 2.0, 0, 0);
            case {'q','escape'}
                setappdata(fig,'running',false);
                safeStop(fig); return;
        end
        setappdata(fig,'SA',SA_); setappdata(fig,'SB',SB_);
    end

    function onKeyUp(~,evt)
        if ~ishandle(fig), return; end
        SA_ = getappdata(fig,'SA');
        SB_ = getappdata(fig,'SB');
        switch evt.Key
            case 'uparrow',   SA_.keys.up    = 0;
            case 'downarrow', SA_.keys.down  = 0;
            case 'leftarrow', SA_.keys.left  = 0;
            case 'rightarrow',SA_.keys.right = 0;
            case 'w', SB_.keys.up    = 0;
            case 's', SB_.keys.down  = 0;
            case 'a', SB_.keys.left  = 0;
            case 'd', SB_.keys.right = 0;
        end
        setappdata(fig,'SA',SA_); setappdata(fig,'SB',SB_);
    end

    function onClose(src,~)
        setappdata(src,'running',false);
        safeStop(src); delete(src);
    end

    function safeStop(f)
        if ~ishandle(f), return; end
        try
            t_=getappdata(f,'tmr');
            if isvalid(t_), stop(t_); delete(t_); end
        catch
        end
    end

end  %% ── end blimpFormation ──────────────────────────────────────────────────


%% ════════════════════════════════════════════════════════════════════════════
%%  LOCAL FUNCTIONS
%% ════════════════════════════════════════════════════════════════════════════

function S = fmState(x0, y0, z0, psi0)
    S.x=x0; S.y=y0; S.z=z0; S.psi=psi0;
    S.u=0; S.v=0;  % surge, sway
    S.w=0; S.r=0;  % heave, yaw rate
    S.u=0; S.v=0;  % surge, sway
    S.w=0; S.r=0;  % heave, yaw rate
    S.w=0; S.r=0;
    S.keys=struct('up',0,'down',0,'left',0,'right',0);
    S.trail_x=[]; S.trail_y=[]; S.trail_z=[];
    S.frame=0; S.t=0;
    S.prev_epsi=0;   % yaw derivative term for autoLeader
    S.ex_prev=0;     % previous X position error — for Kd term
    S.ey_prev=0;     % previous Y position error — for Kd term
    S.vAx_prev=0;    % previous leader inertial X velocity — for leader accel estimate
    S.vAy_prev=0;    % previous leader inertial Y velocity — for leader accel estimate
    S.dist_fx=0;     % current disturbance X force — carried into rk4step
    S.dist_fy=0;     % current disturbance Y force
    S.dist_on=false; % whether disturbance is currently active
    S.dist_rand_fx=1; % random direction X — set once per ON cycle
    S.dist_rand_fy=0; % random direction Y
    S.dist_prev_active=false; % tracks cycle edge to detect new ON cycle
    S.ldr_scale    = 1.0;  % current leader speed scale (for HUD display)
    % Layer 3 comm-loss state
    S.comm_lost    = false;
    S.t_comm_lost  = 0;   % sim time when comm was first lost
    S.last_A_x     = 0;   % A position snapshot at comm loss
    S.last_A_y     = 0;
    S.last_A_psi   = 0;
end

%% ── Manual inputs (arrow keys or WASD, same physics) ────────────────────────
function inp = manualInputs(S, P)
    F1 = S.keys.right * P.F_step;
    F2 = S.keys.left  * P.F_step;
    Fz = (S.keys.up - S.keys.down) * P.F_vert;
    inp.Fx=F1+F2; inp.Fz=Fz; inp.Mz=(F1-F2)*P.ly;
    inp.F1=F1; inp.F2=F2; inp.F3=abs(Fz);
    inp.dist_fx=0; inp.dist_fy=0; inp.dist_on=false;
end

%% ── Autonomous leader: figure-8 lemniscate path controller ─────────────────
function [inp, xwpt, ywpt] = autoLeader(S, P, dt)
    % Parametric figure-8: x=A*cos(w*t), y=A/2*sin(2*w*t)
    % Lookahead: compute desired position slightly ahead in time
    t_look = S.t + 0.8;   % look 0.8 s ahead
    xwpt = P.path_A * cos(P.path_omega * t_look);
    ywpt = P.path_A/2 * sin(2 * P.path_omega * t_look);
    zwpt = P.path_z;

    % Proportional position controller
    ex =  xwpt - S.x;
    ey =  ywpt - S.y;
    ez =  zwpt - S.z;

    % Desired heading toward waypoint
    psi_des = atan2(ey, ex);
    epsi = psi_des - S.psi;
    % Wrap to [-pi, pi]
    epsi = atan2(sin(epsi), cos(epsi));

    % Project position error onto body surge axis
    dist_horiz = sqrt(ex^2 + ey^2);
    F_surge_raw = P.Kp_surge * dist_horiz * cos(epsi);

    % Yaw moment (PD)
    d_epsi = (epsi - S.prev_epsi) / dt;
    Mz = P.Kp_yaw * epsi + P.Kd_yaw * d_epsi;

    F_heave = P.Kp_heave * ez;

    % Motor allocation: F1+F2=F_surge,  (F1-F2)*ly=Mz
    F1 = clamp((F_surge_raw + Mz/P.ly)/2, -P.F_max, P.F_max);
    F2 = clamp((F_surge_raw - Mz/P.ly)/2, -P.F_max, P.F_max);
    F3 = clamp(F_heave, -P.F_max, P.F_max);

    inp.Fx=F1+F2; inp.Fz=F3; inp.Mz=(F1-F2)*P.ly;
    inp.F1=F1; inp.F2=F2; inp.F3=abs(F3);
end

%% ── Follower controller: position + velocity + safety filter ────────────────
% The core idea: a pure position controller (Kp only) fails because it has
% no knowledge of how fast the gap is changing. By the time a large error
% builds up, B is already behind and has to play catch-up. Two extra terms
% fix this — one reactive, one predictive.
function [inp, des_pos, ctrl_err] = autoFollower(S, L, P)
    dt = 0.025;

    % ── Desired position ─────────────────────────────────────────────────
    % Normal: fixed behind-tail. Layer 3 (comm lost): last known A position
    % so a stationary A doesn't confound CLF — B navigates to where A was,
    % prioritising distance recovery over velocity matching.
    if isfield(S,'comm_lost') && S.comm_lost
        % Layer 3: navigate to last known leader position
        xd = S.last_A_x - P.follow_dist * cos(S.last_A_psi);
        yd = S.last_A_y - P.follow_dist * sin(S.last_A_psi);
    else
        xd = L.x - P.follow_dist * cos(L.psi);
        yd = L.y - P.follow_dist * sin(L.psi);
    end

    % ── Outer zone: nearest cone-edge target ─────────────────────────────
    % When B is outside the ideal zone AND outside the ±60° cone, the
    % behind-tail formation point is on the far side of A. The straight CLF
    % path goes through A, so B arcs around A at constant radius — the
    % geometric latch at the yellow ring. Fix: redirect the CLF to the
    % nearest cone-edge point at formation distance. B can reach this
    % directly without crossing A. Arc correction + inner circumnavigation
    % handle final alignment once B is inside the ideal zone.
    d_AB_clf   = sqrt((S.x-L.x)^2 + (S.y-L.y)^2);
    angle_AB_clf  = atan2(S.y - L.y, S.x - L.x);
    angle_behind_clf = L.psi + pi;
    d_angle_clf = atan2(sin(angle_AB_clf - angle_behind_clf), ...
                        cos(angle_AB_clf - angle_behind_clf));
    if d_AB_clf > P.comm_ideal_hi && abs(d_angle_clf) > P.cone_half_angle
        % Nearest reachable cone edge at formation distance
        nearest_cone_ang = angle_behind_clf + sign(d_angle_clf)*P.cone_half_angle;
        xd = L.x + P.follow_dist * cos(nearest_cone_ang);
        yd = L.y + P.follow_dist * sin(nearest_cone_ang);
    end

    % ── Inner CBF circumnavigation ───────────────────────────────────────
    % When B is close to A and the formation point is on the far side,
    % the straight CLF path goes through A — the CBF and CLF compete and
    % B stalls. Instead, blend the CLF target toward a point on the inner
    % ring that advances incrementally toward the behind-tail angle each
    % tick. CBF pushes radially outward; CLF pulls tangentially around A.
    % They cooperate to produce a curved escape back into the cone.
    h1_clf     = d_AB_clf^2 - P.cbf_d_min^2;
    circ_blend = max(0, min(1, h1_clf / P.cbf_h1_thresh));
    % circ_blend=1: normal CLF target. circ_blend=0: full orbit target.
    if circ_blend < 1
        angle_err_clf = atan2(sin(angle_behind_clf - angle_AB_clf), ...
                              cos(angle_behind_clf - angle_AB_clf));
        % Orbit target: 30% step toward behind-tail per correction, on ring
        r_orbit    = P.cbf_d_min * 1.5;  % safely outside inner boundary
        angle_orb  = angle_AB_clf + 0.30 * angle_err_clf;
        xd_orb = L.x + r_orbit * cos(angle_orb);
        yd_orb = L.y + r_orbit * sin(angle_orb);
        % Blend smoothly between orbit and normal target
        xd = circ_blend * xd + (1-circ_blend) * xd_orb;
        yd = circ_blend * yd + (1-circ_blend) * yd_orb;
    end

    zd = L.z;
    des_pos.x=xd; des_pos.y=yd; des_pos.z=zd;

    % ── Cone geometry — used for h3 CBF and soft arc correction ─────────────
    angle_AB     = atan2(S.y - L.y, S.x - L.x);
    angle_behind = L.psi + pi;
    % Angular displacement from behind-tail — signed, wrapped to [-pi, pi]
    d_angle = atan2(sin(angle_AB - angle_behind), cos(angle_AB - angle_behind));
    % h3: cone barrier — positive = inside cone, negative = outside cone
    h3_cone = P.cone_half_angle^2 - d_angle^2;

    % ── Soft arc correction — pushes B tangentially back into cone ───────────
    % Hard cutoff at comm_ideal_hi (6.25m): full arc correction inside the
    % ideal zone, zero outside it. A gradual fade between ideal and yellow
    % created a partial-active boundary at 7.5m — arc correction jumped from
    % zero to 56-80% immediately as B crossed inward, latching B onto a
    % tangential arc right at the yellow ring. Hard cutoff eliminates that.
    % Outside the ideal zone, CLF + circumnavigation are the correct recovery
    % mechanisms — arc correction is not needed and actively harmful there.
    d_AB_arc = sqrt((S.x-L.x)^2 + (S.y-L.y)^2);
    cone_threshold = P.cone_warn_frac * P.cone_half_angle;
    arc_error = abs(d_angle) - cone_threshold;
    if arc_error > 0 && d_AB_arc <= P.comm_ideal_hi
        tang_x =  sin(angle_AB) * sign(d_angle);
        tang_y = -cos(angle_AB) * sign(d_angle);
        arc_scale = P.Kp * arc_error * P.follow_dist;
        arc_fx = arc_scale * tang_x;
        arc_fy = arc_scale * tang_y;
    else
        arc_fx = 0; arc_fy = 0;
    end

    % How far off is B from that spot right now?
    ex = xd - S.x;
    ey = yd - S.y;
    ez = zd - S.z;

    % Reactive term: how fast is the gap currently opening or closing?
    % Computed by differencing the position error between this tick and last.
    % If the gap is growing fast, this pushes harder before it gets worse.
    dex = (ex - S.ex_prev) / dt;
    dey = (ey - S.ey_prev) / dt;

    % Predictive term: match velocity of the FORMATION POINT.
    % Full kinematic velocity including sway v (Eq 5.27):
    %   world_x = u*cos(psi) - v*sin(psi)
    %   world_y = u*sin(psi) + v*cos(psi)
    % Formation point adds rotational sweep: follow_dist * rA.
    vBx = S.u*cos(S.psi) - S.v*sin(S.psi);
    vBy = S.u*sin(S.psi) + S.v*cos(S.psi);
    vForm_x = L.u*cos(L.psi) - L.v*sin(L.psi) + P.follow_dist*sin(L.psi)*L.r;
    vForm_y = L.u*sin(L.psi) + L.v*cos(L.psi) - P.follow_dist*cos(L.psi)*L.r;
    dvx = vForm_x - vBx;
    dvy = vForm_y - vBy;

    % Add all three contributions in inertial frame.
    % Keeping them separate until here ensures units stay consistent
    % (each term independently produces Newtons before being summed).
    % Kff scales to zero when B is outside the safe zone.
    % During recovery (d > 2.0m), Kff interprets B's inward surge as a
    % speed mismatch against the stationary leader and brakes it — exactly
    % the stall the user observed. Kff is only meaningful during steady
    % following; during recovery, Kp and Kd alone should drive convergence.
    h2_ctrl   = P.cbf_d_max^2 - (S.x-L.x)^2 - (S.y-L.y)^2;
    kff_scale = max(0, min(1, h2_ctrl / P.cbf_h2_thresh));
    Fx_inertial = P.Kp*ex + P.Kd*dex + kff_scale*P.Kff*dvx + arc_fx;
    Fy_inertial = P.Kp*ey + P.Kd*dey + kff_scale*P.Kff*dvy + arc_fy;

    % Altitude is handled separately — no horizontal-vertical coupling
    % on this vehicle so a simple proportional controller is sufficient.
    F_heave = P.Kp_heave * ez;

    % Before sending forces to the motors, run them through the safety filter.
    % The filter checks whether the commanded force would violate either
    % communication boundary, and if so finds the closest safe alternative.
    % If the filter is off or unavailable, forces pass through unchanged.
    if P.cbf_enabled
        [Fx_safe, Fy_safe, cbf_info] = ...
            clf_cbf_filter(Fx_inertial, Fy_inertial, S, L, [xd;yd], P, h3_cone);
        inp.cbf_h1     = cbf_info.h1;
        inp.cbf_h2     = cbf_info.h2;
        inp.cbf_h3     = cbf_info.h3;
        inp.cbf_active = cbf_info.active;
        inp.clf_slack  = cbf_info.slack;
        inp.qp_status  = cbf_info.status;
    else
        inp.cbf_h1=NaN; inp.cbf_h2=NaN; inp.cbf_h3=NaN;
        inp.clf_slack=0; inp.qp_status='DISABLED';
    end

    % Project inertial force to body frame with crab angle compensation.
    %
    % Problem without crab: when sway v pushes B sideways, the nose still
    % points at the target — but the actual velocity vector is offset by the
    % sideslip angle beta = atan2(v, u). The blimp surges with its nose at
    % the target but moves perpendicular to it, producing the outer-arc path.
    %
    % Fix: use effective heading psi_eff = psi + beta when computing the
    % body-frame projection. This generates a yaw moment that aims the
    % VELOCITY VECTOR at the target rather than the nose — the blimp crabs
    % into the sway so its actual motion tracks directly toward the target.
    % ── Motor allocation: surge and heading fully separated ──────────────────
    % Previous approach drove Mz from F_lateral (inertial force × sin/cos).
    % When position error was large, F_lateral consumed all motor authority on
    % yaw, leaving zero surge (stall). And with no yaw rate damping, the heading
    % overshot (overspun). Both problems share the same root cause.
    %
    % Fix: explicit heading PD, completely separate from surge.
    %   - Crab moves into psi_des (not into force projection)
    %   - Kd_yaw·r damps yaw rate directly — prevents overshoot
    %   - Surge and heading can no longer starve each other

    % Surge: safe inertial force projected onto true nose direction
    if P.cbf_enabled
        F_surge = Fx_safe*cos(S.psi) + Fy_safe*sin(S.psi);
    else
        beta_nom = atan2(S.v, max(0.05, abs(S.u)));
        F_surge  = Fx_inertial*cos(S.psi+beta_nom) + Fy_inertial*sin(S.psi+beta_nom);
    end

    % Heading PD with Coriolis feedforward:
    %   Kp_yaw: proportional heading correction
    %   Kd_yaw: damps yaw rate — prevents overshoot
    %   Kff_yaw: pre-compensates for sway that Coriolis is about to create
    %     Without this, the disturbance step changes u and Coriolis creates v,
    %     v changes beta, beta changes psi_des, psi_des commands yaw — the nose
    %     wobbles reactively. The feedforward anticipates that beta change and
    %     pre-rotates the nose, absorbing the incoming sway before it accumulates.
    beta    = atan2(S.v, max(0.05, abs(S.u)));    % current sideslip
    % Blend psi_des toward A's center as outer boundary approaches.
    % Normal: track formation point bearing. Near outer boundary: swing
    % heading toward A directly, aborting the spiral before it exits.
    % h2_blend=1 (safe): normal formation bearing.
    % h2_blend=0 (at outer boundary): bearing directly toward A.
    % Surge/CBF unaffected — only Mz changes.
    psi_des_form   = atan2(ey, ex) - beta;
    psi_des    = psi_des_form;  % formation point bearing — blend removed
    epsi    = atan2(sin(psi_des-S.psi), cos(psi_des-S.psi));
    % Expected sway rate from Coriolis + damping (known from current state)
    v_dot_ff  = (-P.m*S.u*S.r - P.Yv*S.v) / P.m_sway;
    beta_dot  = v_dot_ff / max(0.05, abs(S.u));
    Mz = P.Kp_yaw*epsi - P.Kd_yaw*S.r - P.Kff_yaw*beta_dot;

    % Yaw rate soft cap — prevents Coriolis positive feedback.
    % When r is large, m*u*r/m_sway creates sway faster than it can be
    % corrected, causing the heading to spiral.
    if abs(S.r) > P.r_soft_cap
        r_excess = (abs(S.r) - P.r_soft_cap) * sign(S.r);
        Mz = Mz - P.vel_damp_k * P.Iz * r_excess;
    end

    % Outer zone Mz alignment scale.
    % When B is beyond the ideal zone, Mz allocation is driven by how much
    % the motor actually needs to turn vs surge:
    %   Aligned (|nose_r|→1): motor surges directly inward/outward.
    %     Mz is not needed and steals Fx authority — scale toward 0.
    %   Perpendicular (|nose_r|→0): motor has zero radial authority.
    %     Full Mz is needed to turn the nose toward A — scale stays 1.
    % This handles both failure modes: the aligned stall (Mz steals Fx)
    % and the tangent deadlock (Mz=0 leaves B orbiting tangentially).
    % Normal following (d ≤ comm_ideal_hi) is completely unaffected.
    d_mz = sqrt((S.x-L.x)^2 + (S.y-L.y)^2);
    if d_mz > P.comm_ideal_hi
        dx_mz = S.x - L.x;  dy_mz = S.y - L.y;
        nose_r_mz = (dx_mz*cos(S.psi) + dy_mz*sin(S.psi)) / max(d_mz, 0.01);
        mz_align_scale = 1 - abs(nose_r_mz);
        Mz = Mz * mz_align_scale;
    end

    % Inner Mz cap: near the outer comm boundary, preserve minimum
    % surge authority. Kept for the inner zone where heading alignment
    % still matters for motor authority.
    h2_ctrl   = P.cbf_d_max^2 - (S.x-L.x)^2 - (S.y-L.y)^2;
    if P.cbf_enabled && h2_ctrl < P.cbf_h2_thresh
        Mz_cap = (2*P.F_max - 0.60*P.F_max) * P.ly;
        h2_frac_mz = max(0, min(1, h2_ctrl / P.cbf_h2_thresh));
        Mz_limit = Mz_cap + h2_frac_mz * (10.0 - Mz_cap);
        Mz = max(-Mz_limit, min(Mz_limit, Mz));
    end

    % Split the yaw moment into differential motor commands,
    % then clamp everything to physical motor limits.
    F1 = clamp((F_surge + Mz/P.ly)/2, -P.F_max, P.F_max);
    F2 = clamp((F_surge - Mz/P.ly)/2, -P.F_max, P.F_max);
    F3 = clamp(F_heave,                -P.F_max, P.F_max);

    inp.Fx=F1+F2; inp.Fz=F3; inp.Mz=(F1-F2)*P.ly;
    inp.F1=F1; inp.F2=F2; inp.F3=abs(F3);

    % Store this tick's errors so next tick can compute the derivative term.
    inp.ex_prev = ex;
    inp.ey_prev = ey;
    ctrl_err = [ex, ey, ez];
end

%% ── RK4 integrator ──────────────────────────────────────────────────────────
function S = rk4step(S, inp, P, dt)
    % Full body-frame model from thesis Chapter 5 (Eq 5.2).
    % State: u (surge), v (sway), w (heave), r (yaw rate)
    %
    % Key physics:
    %  - m_surge ≠ m_sway (Xu_dot ≠ Yv_dot from added mass, Table 6.3)
    %  - Coriolis: turning at rate r while surging creates sway v = -m*u*r/m_sway
    %  - Kinematics: x_dot = u*cos(ψ) - v*sin(ψ)  (v contributes laterally)
    %  - Disturbance projects into body-frame surge+sway — no separate drift states
    %  - No lateral motor: sway is purely passive (Coriolis + drag + disturbance)

    if ~isfield(S,'v'),         S.v=0; end
    if ~isfield(inp,'dist_fx'), inp.dist_fx=0; inp.dist_fy=0; end

    function d = deriv(s, inp, P)
        % Project world-frame disturbance into body frame
        dist_surge =  inp.dist_fx*cos(s.psi) + inp.dist_fy*sin(s.psi);
        dist_sway  = -inp.dist_fx*sin(s.psi) + inp.dist_fy*cos(s.psi);

        % Surge (Eq 5.2, row 1): motor + Coriolis(+m*v*r) + disturbance - drag
        d.du   = (inp.Fx + P.m*s.v*s.r + dist_surge - P.Xu*s.u) / P.m_surge;

        % Sway (Eq 5.2, row 2): NO motor, Coriolis(-m*u*r) + disturbance - drag
        % This is the centripetal term: turning while surging creates lateral drift
        d.dv   = (-P.m*s.u*s.r + dist_sway - P.Yv*s.v) / P.m_sway;

        % Heave: vertical motor - drag + net buoyancy
        d.dw   = (inp.Fz - P.Zw*s.w) / P.m_heave;

        % Yaw: differential thrust moment - drag
        d.dr   = (inp.Mz - P.Nr*s.r) / P.Iz;

        % Kinematics (Eq 5.27): body-frame to inertial (level flight, small phi/theta)
        % x_dot = u*cos(psi) - v*sin(psi)   <- v now contributes to x
        % y_dot = u*sin(psi) + v*cos(psi)   <- v now contributes to y
        d.dx   = s.u*cos(s.psi) - s.v*sin(s.psi);
        d.dy   = s.u*sin(s.psi) + s.v*cos(s.psi);
        d.dz   = s.w;
        d.dpsi = s.r;
    end

    k1=deriv(S,inp,P);
    s2=struct('x',S.x+dt/2*k1.dx,'y',S.y+dt/2*k1.dy,'z',S.z+dt/2*k1.dz,...
        'psi',S.psi+dt/2*k1.dpsi,'u',S.u+dt/2*k1.du,'v',S.v+dt/2*k1.dv,...
        'w',S.w+dt/2*k1.dw,'r',S.r+dt/2*k1.dr);
    k2=deriv(s2,inp,P);
    s3=struct('x',S.x+dt/2*k2.dx,'y',S.y+dt/2*k2.dy,'z',S.z+dt/2*k2.dz,...
        'psi',S.psi+dt/2*k2.dpsi,'u',S.u+dt/2*k2.du,'v',S.v+dt/2*k2.dv,...
        'w',S.w+dt/2*k2.dw,'r',S.r+dt/2*k2.dr);
    k3=deriv(s3,inp,P);
    s4=struct('x',S.x+dt*k3.dx,'y',S.y+dt*k3.dy,'z',S.z+dt*k3.dz,...
        'psi',S.psi+dt*k3.dpsi,'u',S.u+dt*k3.du,'v',S.v+dt*k3.dv,...
        'w',S.w+dt*k3.dw,'r',S.r+dt*k3.dr);
    k4=deriv(s4,inp,P);
    S.x   = S.x   + dt/6*(k1.dx  +2*k2.dx  +2*k3.dx  +k4.dx);
    S.y   = S.y   + dt/6*(k1.dy  +2*k2.dy  +2*k3.dy  +k4.dy);
    S.z   = S.z   + dt/6*(k1.dz  +2*k2.dz  +2*k3.dz  +k4.dz);
    S.psi = S.psi + dt/6*(k1.dpsi+2*k2.dpsi+2*k3.dpsi+k4.dpsi);
    S.u   = S.u   + dt/6*(k1.du  +2*k2.du  +2*k3.du  +k4.du);
    S.v   = S.v   + dt/6*(k1.dv  +2*k2.dv  +2*k3.dv  +k4.dv);
    S.w   = S.w   + dt/6*(k1.dw  +2*k2.dw  +2*k3.dw  +k4.dw);
    S.r   = S.r   + dt/6*(k1.dr  +2*k2.dr  +2*k3.dr  +k4.dr);
    if isfield(S,'prev_epsi'), S.prev_epsi=S.r; end
end


%% ── Disturbance generator ────────────────────────────────────────────────────
function [inp, SB] = applyDisturbance(inp, SB, SA, P, t)
    inp.dist_fx = 0;  inp.dist_fy = 0;  inp.dist_on = false;
    if ~P.dist_enabled || t < P.dist_t_start
        SB.dist_on = false;
        SB.dist_prev_active = false;
        return;
    end
    cycle_t   = mod(t - P.dist_t_start, P.dist_t_on + P.dist_t_off);
    is_active = cycle_t < P.dist_t_on;
    if is_active
        % Detect rising edge of ON cycle — draw new random direction once
        if ~SB.dist_prev_active
            if P.dist_direction == 0
                % Random unit vector, fixed for this cycle
                ang = rand() * 2 * pi;
                SB.dist_rand_fx = cos(ang);
                SB.dist_rand_fy = sin(ang);
            else
                % Fixed radial direction: compute outward unit vector
                dx = SB.x - SA.x;  dy = SB.y - SA.y;
                d  = sqrt(dx^2 + dy^2);
                if d < 0.01, dx=1; dy=0; d=1; end
                SB.dist_rand_fx = P.dist_direction * (dx/d);
                SB.dist_rand_fy = P.dist_direction * (dy/d);
            end
        end
        inp.dist_fx = P.dist_mag * SB.dist_rand_fx;
        inp.dist_fy = P.dist_mag * SB.dist_rand_fy;
        inp.dist_on = true;
    end
    SB.dist_prev_active = is_active;
    SB.dist_on  = is_active;
    SB.dist_fx  = inp.dist_fx;
    SB.dist_fy  = inp.dist_fy;
end

%% ── Clamp helper ─────────────────────────────────────────────────────────────
function v = iif(cond, a, b)
    if cond, v=a; else, v=b; end
end

function v = clamp(v, lo, hi)
    v = max(lo, min(hi, v));
end

%% ── Draw everything ─────────────────────────────────────────────────────────
function drawFormation(H, SA, inpA, SB, inpB, P, E0, mode, xwpt, ywpt, des_pos, ctrl_err)
    GRN=[0.17 1.00 0.56]; CYN=[0.10 0.85 0.95]; CYN2=[0.05 0.55 0.70];
    a=P.a; b=P.b; c=P.c; N=E0.n; gw=0.16; gh=0.08; gw3=0.14; gd3=0.22; gh3=0.09;
    t=linspace(0,2*pi,40); th_c=linspace(0,2*pi,80);

    % Draw both agents
    drawBlimp(H.hA_EnvS,H.hA_GndS,H.hA_DirS,H.hA_TrailS, ...
              H.hA_EnvT,H.hA_DirT,H.hA_TrailT,H.hA_LblT, ...
              H.hA_EnvF,H.hA_TrailF, ...
              H.hA_Surf,H.hA_Gond3,H.hA_Fin3L,H.hA_Fin3R,H.hA_Fin3T, ...
              H.hA_Dir3,H.hA_Trail3, SA, E0, P);

    drawBlimp(H.hB_EnvS,H.hB_GndS,H.hB_DirS,H.hB_TrailS, ...
              H.hB_EnvT,H.hB_DirT,H.hB_TrailT,H.hB_LblT, ...
              H.hB_EnvF,H.hB_TrailF, ...
              H.hB_Surf,H.hB_Gond3,H.hB_Fin3L,H.hB_Fin3R,H.hB_Fin3T, ...
              H.hB_Dir3,H.hB_Trail3, SB, E0, P);

    % Fixed view centred on A — shows full comm zone at lab scale
    % Span = comm_max_red + 2m margin on each side
    hw = P.comm_max_red + 2;  % half-width [m]
    mz = SA.z;
    xlim(H.axS,[SA.x-hw  SA.x+hw]);  ylim(H.axS,mz+[-1.5 2.5]);
    xlim(H.axT,[SA.x-hw  SA.x+hw]);  ylim(H.axT,[SA.y-hw  SA.y+hw]);
    xlim(H.axF,[SA.y-hw  SA.y+hw]);  ylim(H.axF,mz+[-1 2.5]);
    xlim(H.ax3,[SA.x-hw  SA.x+hw]);  ylim(H.ax3,[SA.y-hw  SA.y+hw]);
    zlim(H.ax3,mz+[-1.5 2.5]);

    % ── Disturbance status on mode banner ─────────────────────────────────────
    if P.dist_enabled
        if SB.dist_on
            dist_str = 'DISTURBED';
            dist_col = [1.0 0.4 0.1];
        elseif SB.t < P.dist_t_start
            dist_str = sprintf('DIST IN %.1fs', P.dist_t_start - SB.t);
            dist_col = [0.6 0.6 0.3];
        else
            dist_str = 'CALM';
            dist_col = [0.3 0.8 0.4];
        end
        set(H.svModeHint,'String',dist_str,'Color',dist_col);
    end

    % ── Mode-specific overlays ────────────────────────────────────────────────
    if mode == 1
        % Show path preview + current waypoint
        set(H.hPathPrev,'Visible','on');
        set(H.hPathPrev3,'Visible','on');
        set(H.hWpt_T,'XData',xwpt,'YData',ywpt,'Visible','on');
        set(H.hWpt_3,'XData',xwpt,'YData',ywpt,'ZData',P.path_z,'Visible','on');
        set(H.hDesPos_T,'Visible','off');
        set(H.hDesPos_3,'Visible','off');
        set(H.hDesLine_T,'Visible','off');
        set(H.svModeBanner,'String','MODE 1','Color',[0.17 1.00 0.56]);
        set(H.svModeDesc,'String','AUTO LEADER','Color',[0.17 1.00 0.56]);
        set(H.svModeHint,'String','B: WASD  A: autonomous');
    else
        % Show desired formation position (where follower aims)
        set(H.hPathPrev,'Visible','off');
        set(H.hPathPrev3,'Visible','off');
        set(H.hWpt_T,'Visible','off');
        set(H.hWpt_3,'Visible','off');
        if ~isnan(des_pos.x)
            set(H.hDesPos_T,'XData',des_pos.x,'YData',des_pos.y,'Visible','on');
            set(H.hDesPos_3,'XData',des_pos.x,'YData',des_pos.y,'ZData',des_pos.z,'Visible','on');
            set(H.hDesLine_T,'XData',[SB.x des_pos.x SA.x], ...
                'YData',[SB.y des_pos.y SA.y],'Visible','on');
        end
        set(H.svModeBanner,'String','MODE 2','Color',CYN);
        set(H.svModeDesc,'String','AUTO FOLLOWER','Color',CYN);
        set(H.svModeHint,'String','A: Arrows  B: autonomous');
        % Controller error readout
        cbf_str = cbf_status_str(inpB.cbf_h1, inpB.cbf_h2, inpB.cbf_active);
        if isfield(inpB,'qp_status'), qs=inpB.qp_status; else, qs=''; end
        set(H.svCtrlErr,'String',sprintf('ex%+.2f ey%+.2f ez%+.2f',...
            ctrl_err(1),ctrl_err(2),ctrl_err(3)));
    end

    % ── System status column update ───────────────────────────────────────
    ls = SB.ldr_scale;
    % Determine layer
    dAB = sqrt((SB.x-SA.x)^2+(SB.y-SA.y)^2);
    if dAB > P.cbf_d_max
        elapsed_l3 = SB.t - SB.t_comm_lost;
        if elapsed_l3 >= P.layer3_timeout
            layer_str = 'L3 RES'; layer_col = [0.9 0.5 0.1];
        else
            layer_str = sprintf('L3 %ds',round(P.layer3_timeout-elapsed_l3));
            layer_col = [0.9 0.2 0.2];
        end
    elseif ls < 0.99
        layer_str = 'L2'; layer_col = [0.9 0.8 0.1];
    else
        layer_str = 'L1'; layer_col = [0.3 0.9 0.3];
    end
    spd_col = [1-ls, 0.3+0.6*ls, 0.1];  % red→green as scale increases
    set(H.svLayer,  'String',layer_str,         'Color',layer_col);
    set(H.svSpdScl, 'String',sprintf('%.2f',ls),'Color',spd_col);
    % Predicted distance (recompute here for display)
    vBx_d = SB.u*cos(SB.psi)-SB.v*sin(SB.psi);
    vBy_d = SB.u*sin(SB.psi)+SB.v*cos(SB.psi);
    vAx_d = SA.u*cos(SA.psi)-SA.v*sin(SA.psi);
    vAy_d = SA.u*sin(SA.psi)+SA.v*cos(SA.psi);
    ux_d  = (SB.x-SA.x)/max(dAB,0.01);
    uy_d  = (SB.y-SA.y)/max(dAB,0.01);
    v_out_d = (vBx_d-vAx_d)*ux_d+(vBy_d-vAy_d)*uy_d;
    a_wc_d  = P.dist_max_force/P.m_sway*(v_out_d>=0);
    d_pr    = dAB+v_out_d*P.leader_T2+0.5*a_wc_d*P.leader_T2^2;
    set(H.svPredD,  'String',sprintf('%.1fm',d_pr));
    if isfield(inpB,'cbf_h1') && ~isnan(inpB.cbf_h1)
        set(H.svH1,'String',sprintf('%.1f',inpB.cbf_h1));
        set(H.svH2,'String',sprintf('%.1f',inpB.cbf_h2));
    end
    if isfield(inpB,'qp_status')
        cbf_on = inpB.cbf_active;
        set(H.svCBFsys,'String',inpB.qp_status, ...
            'Color', [0.3+(cbf_on*0.6) 0.9-(cbf_on*0.6) 0.3-(cbf_on*0.2)]);
    end
    dist_str2 = 'OFF'; dist_col2 = [0.3 0.5 0.3];
    if SB.dist_on
        if P.dist_direction == 0
            ang_deg = round(atan2(SB.dist_rand_fy, SB.dist_rand_fx)*180/pi);
            dist_str2 = sprintf('ON %d°', ang_deg);
        elseif P.dist_direction > 0
            dist_str2 = 'ON OUT';
        else
            dist_str2 = 'ON IN';
        end
        dist_col2 = [0.9 0.3 0.1];
    end
    set(H.svDistSys,'String',dist_str2,'Color',dist_col2);

    % ── Disturbance arrow on top view ─────────────────────────────────────────
    if SB.dist_on
        % Arrow starts at B, points in disturbance direction, length = force mag
        arrow_scale = 1.2;   % visual scale so arrow is readable
        set(H.hDistArrow,'XData',SB.x,'YData',SB.y,...
            'UData',SB.dist_fx*arrow_scale,'VData',SB.dist_fy*arrow_scale,...
            'Visible','on');
        set(H.hDistLabel,'Position',[SB.x+SB.dist_fx*1.4, SB.y+SB.dist_fy*1.4],...
            'String',sprintf('DIST %.2fN %s',P.dist_mag, ...
                iif(P.dist_direction>0,'OUT', ...
                iif(P.dist_direction<0,'IN','RND'))),'Visible','on');
    else
        set(H.hDistArrow,'Visible','off');
        set(H.hDistLabel,'Visible','off');
    end


    % ── Formation cone visualisation ──────────────────────────────────────────
    % Compute cone geometry from SA heading and current A-B angle
    cone_r        = P.follow_dist;
    angle_behind_d= SA.psi + pi;
    angle_AB_d    = atan2(SB.y - SA.y, SB.x - SA.x);
    d_angle_d     = atan2(sin(angle_AB_d - angle_behind_d), ...
                          cos(angle_AB_d - angle_behind_d));

    % Cone boundary angles
    cone_lo = angle_behind_d - P.cone_half_angle;
    cone_hi = angle_behind_d + P.cone_half_angle;
    th_arc  = linspace(cone_lo, cone_hi, 50);

    % Arc points at follow_dist
    arc_x = SA.x + cone_r * cos(th_arc);
    arc_y = SA.y + cone_r * sin(th_arc);

    % Filled wedge: centre → arc → back
    wedge_x = [SA.x, arc_x, SA.x];
    wedge_y = [SA.y, arc_y, SA.y];

    % Cone edge lines
    set(H.hConeEdge1, 'XData', [SA.x SA.x+cone_r*cos(cone_lo)], ...
                      'YData', [SA.y SA.y+cone_r*sin(cone_lo)]);
    set(H.hConeEdge2, 'XData', [SA.x SA.x+cone_r*cos(cone_hi)], ...
                      'YData', [SA.y SA.y+cone_r*sin(cone_hi)]);
    set(H.hConeFill,  'XData', wedge_x, 'YData', wedge_y);
    set(H.hConeArc,   'XData', arc_x,   'YData', arc_y);

    % B's position projected onto the follow-distance arc
    b_arc_x = SA.x + cone_r * cos(angle_AB_d);
    b_arc_y = SA.y + cone_r * sin(angle_AB_d);
    set(H.hConeBMark, 'XData', b_arc_x, 'YData', b_arc_y);

    % Colour everything by how close B is to the cone boundary
    cone_frac = abs(d_angle_d) / P.cone_half_angle;  % 0=behind-tail, 1=boundary
    if cone_frac < 0.75
        cone_col = [0.20 1.00 0.35];   % green — well inside
    elseif cone_frac < 1.0
        cone_col = [1.00 0.80 0.10];   % yellow — approaching limit
    else
        cone_col = [1.00 0.25 0.25];   % red — outside cone
    end
    set(H.hConeEdge1, 'Color', cone_col);
    set(H.hConeEdge2, 'Color', cone_col);
    set(H.hConeArc,   'Color', cone_col);
    set(H.hConeBMark, 'Color', [0.10 0.85 0.95], 'MarkerFaceColor', cone_col);
    % Tint the fill to match
    if cone_frac >= 1.0
        set(H.hConeFill, 'FaceColor', [0.5 0.08 0.08]);
    elseif cone_frac >= 0.75
        set(H.hConeFill, 'FaceColor', [0.4 0.35 0.05]);
    else
        set(H.hConeFill, 'FaceColor', [0.1 0.4 0.15]);
    end

    % ── Comm zone rings (centred on A) ────────────────────────────────────────
    ab_dist=sqrt((SA.x-SB.x)^2+(SA.y-SB.y)^2+(SA.z-SB.z)^2);
    zmid=(SA.z+SB.z)/2;

    if ab_dist < P.comm_min_red
        zStr='DANGER CLOSE'; abCol=[1.0 0.20 0.20]; ls='-';  lw=2.5;
    elseif ab_dist < P.comm_min_yellow
        zStr='TOO CLOSE';    abCol=[1.0 0.80 0.10]; ls='--'; lw=1.8;
    elseif ab_dist <= P.comm_ideal_hi
        zStr='IDEAL';        abCol=[0.20 1.0 0.40]; ls='-';  lw=1.8;
    elseif ab_dist < P.comm_max_yellow
        zStr='TOO FAR';      abCol=[1.0 0.80 0.10]; ls='--'; lw=1.8;
    elseif ab_dist < P.comm_max_red
        zStr='OUTER WARN';   abCol=[1.0 0.45 0.10]; ls='--'; lw=2.0;
    else
        zStr='COMM LOST';    abCol=[1.0 0.20 0.20]; ls=':';  lw=1.2;
    end

    set(H.hLink_T,'XData',[SA.x SB.x],'YData',[SA.y SB.y],'Color',abCol,'LineStyle',ls,'LineWidth',lw);
    set(H.hLink_3,'XData',[SA.x SB.x],'YData',[SA.y SB.y],'ZData',[SA.z SB.z],'Color',abCol,'LineStyle',ls,'LineWidth',lw);

    set(H.hCommInner,'XData',SA.x+P.comm_min_red*cos(th_c),'YData',SA.y+P.comm_min_red*sin(th_c));
    set(H.hCommLo,   'XData',SA.x+P.comm_min_yellow*cos(th_c),'YData',SA.y+P.comm_min_yellow*sin(th_c));
    set(H.hCommHi,   'XData',SA.x+P.comm_max_yellow*cos(th_c),'YData',SA.y+P.comm_max_yellow*sin(th_c));
    set(H.hCommOuter,'XData',SA.x+P.comm_max_red*cos(th_c),'YData',SA.y+P.comm_max_red*sin(th_c));
    set(H.hCommOuter3,'XData',SA.x+P.comm_max_red*cos(th_c), ...
        'YData',SA.y+P.comm_max_red*sin(th_c),'ZData',zmid*ones(1,80));
    set(H.hCommHi3,  'XData',SA.x+P.comm_max_yellow*cos(th_c), ...
        'YData',SA.y+P.comm_max_yellow*sin(th_c),'ZData',zmid*ones(1,80));

    % Zone pointer
    ze=[0,P.comm_min_red,P.comm_min_yellow,P.comm_ideal_hi,P.comm_max_yellow,P.comm_max_red];
    dc=min(ab_dist,P.comm_max_red);
    seg=min(find(dc>=ze,1,'last'),5);
    frac=(dc-ze(seg))/max(ze(seg+1)-ze(seg),1e-6);
    px=0.05+(seg-1)*0.18+frac*0.18;
    set(H.svZonePtr,'XData',px,'YData',0.683,'Color',abCol,'MarkerFaceColor',abCol);
    set(H.svABDist, 'String',sprintf('%.2f m',ab_dist),'Color',abCol);
    set(H.svCommZone,'String',zStr,'Color',abCol);

    % State readouts A + B
    % uA unused — total speed already in vA(5)
    vA=[SA.x,SA.y,SA.z,SA.psi*180/pi,sqrt(SA.u^2+SA.v^2),SA.w,SA.r];
    % uB unused — total speed already in vB(5)
    vB=[SB.x,SB.y,SB.z,SB.psi*180/pi,sqrt(SB.u^2+SB.v^2),SB.w,SB.r];
    for i=1:7
        set(H.svA(i),'String',sprintf('%+.2f',vA(i)));
        set(H.svB(i),'String',sprintf('%+.2f',vB(i)));
    end
end


%% ── CLF-CBF safety filter ────────────────────────────────────────────────────
% Every tick, before the follower's computed forces reach the motors, they
% pass through here. The question asked is simple: would those forces push B
% toward a boundary it shouldn't cross? If yes, what is the smallest change
% that makes them safe?
%
% The answer is a small optimisation problem (QP) solved in ~0.1 ms.
% Two barrier functions define the no-go zones — one prevents B from getting
% too close (collision risk), one prevents it from drifting too far (comm loss).
% A third constraint, the CLF, ensures B is always actively converging toward
% the formation position rather than just hovering near the boundary.
%
% No slack is used here since there are no static obstacles. If the CLF and
% CBF constraints genuinely conflict (e.g. motor saturation during a sharp
% turn), the CLF is dropped and safety takes priority.
%
% Requires the MATLAB Optimization Toolbox. If unavailable, forces pass
% through unchanged and the HUD shows NO TOOLBOX.

function [Fx_s, Fy_s, info] = clf_cbf_filter(Fx_nom, Fy_nom, S, L, des_xy, P, h3_cone)
    if nargin < 7, h3_cone = P.cone_half_angle^2; end  % default: well inside cone

    dt = 0.025;
    m  = P.m_surge;

    % Passthrough guard: if well inside both safe zones, the nominal
    % control already satisfies the CBF constraints — skip the QP entirely.
    % This prevents the QP from modifying normal formation-keeping forces
    % and only activates as a genuine safety backstop near boundaries.
    dx_pt = S.x - L.x;  dy_pt = S.y - L.y;
    d_sq_pt = dx_pt^2 + dy_pt^2;
    h1_pt = d_sq_pt - P.cbf_d_min^2;
    h2_pt = P.cbf_d_max^2 - d_sq_pt;
    % Broadside threat check: is B's heading poorly aligned against the
    % outward radial AND far enough out that a disturbance is dangerous?
    d_pt = max(sqrt(d_sq_pt), 0.01);
    nose_r_pt = (dx_pt*cos(S.psi) + dy_pt*sin(S.psi)) / d_pt;
    broadside_threat = (abs(nose_r_pt) < P.cbf_broadside_thresh) && ...
                       (d_pt > P.cbf_broadside_d_min);

    if h1_pt > P.cbf_h1_thresh && h2_pt > P.cbf_h2_thresh && ~broadside_threat
        % Well inside safe zone and not broadside — pass through unchanged
        Fx_s = Fx_nom;  Fy_s = Fy_nom;
        info.h1=h1_pt; info.h2=h2_pt; info.h3=h3_cone; info.V=0; info.V_dot=0;
        info.active=false; info.slack=0; info.h3_slack=0; info.status='NOMINAL';
        return;
    end

    % Work in acceleration space (divide by mass once here, multiply back at end).
    % This keeps the QP decision variables dimensionally clean.
    ax_nom = Fx_nom / m;
    ay_nom = Fy_nom / m;

    % Convert body-frame speeds to inertial X/Y components.
    % Follower velocity = motor surge + disturbance drift (vdx, vdy).
    % The filter MUST see both: using only motor velocity caused the QP to
    % compute wrong h1_dot/h2_dot, making corrections that pushed B toward
    % boundaries rather than away — the root cause of CBF-on erratic behaviour.
    vBx = S.u*cos(S.psi) - S.v*sin(S.psi);
    vBy = S.u*sin(S.psi) + S.v*cos(S.psi);
    vAx = L.u*cos(L.psi) - L.v*sin(L.psi);
    vAy = L.u*sin(L.psi) + L.v*cos(L.psi);

    % Leader acceleration estimated by finite difference on stored velocity.
    % Needed because the barrier RHS terms include leader acceleration —
    % without it the constraint is wrong whenever the leader is accelerating.
    aAx = (vAx - L.vAx_prev) / dt;
    aAy = (vAy - L.vAy_prev) / dt;

    % Relative position and velocity — everything is B minus A so that
    % positive values mean B is ahead/faster, negative means behind/slower.
    dx   = S.x - L.x;
    dy   = S.y - L.y;
    d_sq = dx^2 + dy^2;
    dvx  = vBx - vAx;
    dvy  = vBy - vAy;
    dv_sq= dvx^2 + dvy^2;
    dp_dv= dx*dvx + dy*dvy;   % relative position dotted with relative velocity

    % The two barrier functions. Both are positive when safe, zero at the
    % boundary, negative if violated. Using squared distance avoids a square
    % root and keeps the derivatives simple.
    % h1: are we safely outside the danger-close inner circle?
    % h2: are we safely inside the comm-loss outer circle? (sign inverts)
    h1     =  d_sq - P.cbf_d_min^2;
    h1_dot =  2 * dp_dv;    % positive = gap growing = moving away from A
    h2     =  P.cbf_d_max^2 - d_sq;
    h2_dot = -2 * dp_dv;    % positive = gap shrinking = moving toward A

    % The formation target isn't a fixed point — it moves with the leader.
    % When the leader turns, the point behind its tail sweeps an arc, adding
    % a rotational component on top of the translational velocity.
    % Getting this right is what fixed the CLF being invisible in testing.
    % Velocity of desired formation position (moving with leader's rotation)
    vdes_x = vAx + P.follow_dist * sin(L.psi) * L.r;
    vdes_y = vAy - P.follow_dist * cos(L.psi) * L.r;

    % CLF: V measures how far B is from where it should be.
    % The constraint forces V to decrease — B must always be converging.
    epx  = S.x - des_xy(1);
    epy  = S.y - des_xy(2);
    V    = epx^2 + epy^2;

    % V_dot must use the velocity of B relative to the moving target, not
    % just B's raw velocity. Otherwise a stationary B looks like it's
    % converging when the target is moving away, which was the original bug.
    V_dot = 2*epx*(vBx - vdes_x) + 2*epy*(vBy - vdes_y);
    ev_sq = (vBx-vdes_x)^2 + (vBy-vdes_y)^2;

    % ── Uncontrolled acceleration of B (Coriolis + kinematic rotation) ──────
    % The motor only produces surge Fx along the nose. Everything else that
    % contributes to B's inertial acceleration is uncontrolled:
    %   - Coriolis surge term:  +m*v*r in body surge equation
    %   - Centripetal sway:     -m*u*r in body sway equation (no motor)
    %   - Kinematic rotation:   heading angle sweeping body velocities
    % These must be moved to the RHS so the QP commands the motor to
    % compensate — otherwise the CBF guarantee is invalidated.

    % Uncontrolled surge acceleration (Coriolis + damping, no Fx)
    a_surge_unc = (P.m*S.v*S.r - P.Xu*S.u) / P.m_surge;
    % Uncontrolled sway acceleration (centripetal + damping, no motor)
    a_sway_unc  = (-P.m*S.u*S.r - P.Yv*S.v) / P.m_sway;
    % Convert to inertial frame (full kinematic derivative including rotation)
    a_unc_x = a_surge_unc*cos(S.psi) - a_sway_unc*sin(S.psi) ...
              - S.u*sin(S.psi)*S.r - S.v*cos(S.psi)*S.r;
    a_unc_y = a_surge_unc*sin(S.psi) + a_sway_unc*cos(S.psi) ...
              + S.u*cos(S.psi)*S.r - S.v*sin(S.psi)*S.r;
    % Projection of uncontrolled acceleration onto the A-B axis
    a_unc_dp = dx*a_unc_x + dy*a_unc_y;

    % Tangential velocity magnitude relative to A-B axis
    % When B moves tangentially to the comm circle, h2_dot ≈ 0 but the
    % trajectory is curving away. Add a penalty term to the outer barrier
    % RHS so the constraint tightens when tangential speed is high.
    d_ab = max(sqrt(d_sq), 0.01);
    v_tangential_sq = max(0, dv_sq - (dp_dv/d_ab)^2);  % |Δv|^2 - v_radial^2
    % Extra tightening on outer barrier proportional to tangential escape velocity
    tangential_penalty = 0.5 * v_tangential_sq;

    % ── QP setup: 3 variables [ax, ay, delta_clf] ────────────────────────────
    % The QP finds the closest safe inertial acceleration to the nominal.
    % Crab angle correction is applied AFTER the QP in the motor allocation,
    % separately for nominal (with crab) and CBF correction (without crab).
    ax_nom = Fx_nom / m;
    ay_nom = Fy_nom / m;
    H_qp = 2 * diag([1, 1, P.clf_p_slack]);
    f_qp = [-2*ax_nom; -2*ay_nom; 0];

    % h3 cone: computed for display/monitoring only — not in QP constraints
    d_ab_safe = max(sqrt(d_sq), 0.01);

    % Worst-case disturbance margin:
    % We don't know the disturbance direction, but we know its max magnitude.
    % For the inner barrier, worst case is disturbance pointing directly
    % inward (toward A) — maximally reducing h1. Its contribution to the
    % radial acceleration is d_ab * (F_dist_max / m_surge), so we subtract
    % this from rhs1 to force the motor to pre-compensate.
    % For the outer barrier, worst case is disturbance pointing outward —
    % same magnitude, so we subtract the same term from rhs2.
    % Scaling by d_ab_safe is physically correct: a radially-outward force
    % at larger radius has more leverage on the squared-distance barrier.
    dist_margin_inner = 2 * d_ab_safe * (P.dist_max_force / P.m_surge);
    dist_margin_outer = 2 * d_ab_safe * (P.dist_max_force / P.m_sway);

    % Inner-outer coordination: scale inner alpha by remaining outer margin.
    % When B is near the outer boundary, the inner correction is gentler so
    % expulsion velocity never exceeds what the outer CBF can absorb.
    % At d=d_min (inner boundary): frac=1.0, full alpha — catcher has max runway.
    % At d=d_max (outer boundary): frac=0.0, zero alpha — no runway left.
    % The inner and outer barriers are now aware of each other through B's position.
    outer_margin_frac = max(0, min(1, (P.cbf_d_max - d_ab) / ...
                                      (P.cbf_d_max - P.cbf_d_min)));
    % Quadratic fade: correction energy (proportional to alpha^2) scales
    % with frac^2, dropping steeply once B has cleared the inner zone.
    % At midpoint (d=6.5m): linear gives 50% alpha, quadratic gives 25%.
    % Near inner boundary (d=3.5m): both give ~93% — collision avoidance
    % response is preserved where it matters most.
    alpha1_eff = P.cbf_alpha1 * outer_margin_frac^2;
    alpha2_eff = P.cbf_alpha2 * outer_margin_frac^2;

    % Inner barrier — only active when B is within the activation range
    % (h1 < cbf_h1_thresh, d < 4.5m). At larger distances the inner
    % boundary poses no threat, but dist_margin_inner scales with d and
    % was creating a spurious outward constraint that prevented inward
    % recovery. Deactivate when h1 >= h1_thresh (trivially satisfied).
    if h1 < P.cbf_h1_thresh
        rhs1 = 2*dv_sq ...
             - 2*(dx*aAx + dy*aAy) ...
             - 2*a_unc_dp ...
             - dist_margin_inner ...
             + (alpha1_eff+alpha2_eff)*h1_dot ...
             + alpha1_eff*alpha2_eff*h1;
    else
        rhs1 = 1e6;  % inner CBF trivially satisfied — no constraint
    end

    % Outer barrier — hard, with worst-case outward disturbance margin
    rhs2 = -2*dv_sq ...
         + 2*(dx*aAx + dy*aAy) ...
         + 2*a_unc_dp ...
         - dist_margin_outer ...
         - tangential_penalty ...
         + (P.cbf_beta1+P.cbf_beta2)*h2_dot ...
         + P.cbf_beta1*P.cbf_beta2*h2;

    % CLF — soft with delta_clf
    rhs_clf = -(2*ev_sq ...
         + (P.clf_gamma1+P.clf_gamma2)*V_dot ...
         + P.clf_gamma1*P.clf_gamma2*V);

    % Constraint matrix [ax, ay, delta_clf]
    a_max  = P.F_max / m;
    A_ineq = [-2*dx,   -2*dy,   0;   % h1 inner (hard)
               2*dx,    2*dy,   0;   % h2 outer (hard)
               2*epx,   2*epy, -1;   % CLF      (soft delta_clf)
               0,        0,    -1];  % delta_clf >= 0
    b_ineq = [rhs1; rhs2; rhs_clf; 0];
    lb = [-a_max; -a_max; 0];
    ub = [ a_max;  a_max; 1e6];

    % Solve. Warm-start from nominal — usually only a small correction needed.
    % Perturb z0 slightly inside motor bounds — prevents active-set from
    % starting on a bound face, which causes spurious infeasibility when
    % ax_nom is exactly at the motor limit.
    ax0 = max(lb(1)+0.01, min(ub(1)-0.01, ax_nom));
    ay0 = max(lb(2)+0.01, min(ub(2)-0.01, ay_nom));
    z0   = [ax0; ay0; 0];
    % active-set handles bound-active problems correctly.
    % MaxIterations caps compute to prevent simulation stall at 25ms tick.
    opts = optimoptions('quadprog','Display','off', ...
        'Algorithm','active-set','MaxIterations',200);
    ax_s = ax_nom; ay_s = ay_nom;
    cbf_active = false; qp_status = 'NOMINAL';
    h3_slack = 0; clf_slack_val = 0;

    try
        [z_sol, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, ...
                                        [], [], lb, ub, z0, opts);
        if exitflag == 1
            ax_s = z_sol(1); ay_s = z_sol(2);
            clf_slack_val = z_sol(3);
            cbf_active = norm([ax_s-ax_nom, ay_s-ay_nom]) > 0.05;
            if cbf_active, qp_status = 'QP ACTIVE'; end
        else
            [z_sol2, ~, exitflag2] = quadprog( ...
                2*eye(2), [-2*ax_nom;-2*ay_nom], ...
                A_ineq(1:2,1:2), b_ineq(1:2), [], [], lb(1:2), ub(1:2), z0(1:2), opts);
            if exitflag2 == 1
                ax_s = z_sol2(1); ay_s = z_sol2(2);
                cbf_active = true; qp_status = 'CBF ONLY';
            else
                qp_status = 'INFEASIBLE';
                % Direction-aware infeasibility fallback:
                % Outer violation (h2<0): max inward thrust toward A.
                % Inner violation (h1<0): max outward thrust away from A.
                % Both barriers satisfied (numerical QP failure): apply
                %   nominal force — it is provably safe when h1>0 and h2>0.
                %   Previous fallback was velocity damping which gives zero
                %   force when B is stationary, leaving B stuck indefinitely.
                dx_u = (S.x - L.x) / max(d_ab, 0.01);
                dy_u = (S.y - L.y) / max(d_ab, 0.01);
                if h2 < 0       % outside outer boundary
                    ax_s = -a_max * dx_u;
                    ay_s = -a_max * dy_u;
                elseif h1 < 0   % inside inner boundary
                    ax_s =  a_max * dx_u;
                    ay_s =  a_max * dy_u;
                else            % both safe — numerical failure, not true infeasibility
                    ax_s = ax_nom;  % nominal is safe: apply it directly
                    ay_s = ay_nom;
                end
            end
        end
    catch
        qp_status = 'NO TOOLBOX';
    end

    Fx_s = ax_s * m;
    Fy_s = ay_s * m;

    info.h1      = h1;
    info.h2      = h2;
    info.h3      = h3_cone;
    info.V       = V;
    info.V_dot   = V_dot;
    info.active  = cbf_active;
    info.slack   = clf_slack_val;
    info.h3_slack= 0;
    info.status  = qp_status;
end

%% ── CBF zone label for HUD ──────────────────────────────────────────────────
function s = cbf_status_str(h1, h2, active)
    if isnan(h1)
        s = 'CBF OFF';
    elseif h1 < 0
        s = 'CBF! INNER';
    elseif h2 < 0
        s = 'CBF! OUTER';
    elseif active
        s = 'CBF ACTIVE';
    else
        s = 'CBF CLEAR';
    end
end

%% ── Draw one blimp — local function ────────────────────────────────────────
function drawBlimp(hEnvS,hGndS,hDirS,hTrailS, ...
                   hEnvT,hDirT,hTrailT,hLblT, ...
                   hEnvF,hTrailF, ...
                   hSurf,hGond3,hFin3L,hFin3R,hFin3T,hDir3,hTrail3, ...
                   S, E0, P)
    % Simplified marker — readable at lab scale (±15m axes).
    % Body: asymmetric bullet shape (pointed nose, round tail) → heading clear.
    % Velocity quiver: actual world-frame velocity vector → motion direction + magnitude.
    % 3D: vectorized rotation for performance, reduced grid density.

    cp = cos(S.psi);  sp = sin(S.psi);

    % Display scale — 3× physical for visibility at ±15m
    dr_f = 1.20;   % front (nose) semi-axis [m display]
    dr_r = 1.60;   % rear  (tail) semi-axis [m display]
    dr_w = 0.52;   % width semi-axis [m display]

    % ── Bullet/blimp silhouette (front pointed, rear rounded) ─────────────────
    t_f = linspace(-pi/2,  pi/2, 32);   % front arc
    t_r = linspace( pi/2, 3*pi/2, 32);  % rear arc
    x_body = [dr_f*cos(t_f),  dr_r*cos(t_r)];
    y_body = [dr_w*sin(t_f),  dr_w*sin(t_r)];
    % Rotate to heading
    bx_r = x_body*cp - y_body*sp + S.x;
    by_r = x_body*sp + y_body*cp + S.y;

    % ── World-frame velocity for quiver ────────────────────────────────────────
    vBx = S.u*cp - S.v*sp;
    vBy = S.u*sp + S.v*cp;
    v_len = max(sqrt(vBx^2+vBy^2), 0.01);
    v_disp = min(3.0, v_len * 2.5);   % scale: 2.5m display per m/s, cap 3m

    % ── TOP VIEW ───────────────────────────────────────────────────────────────
    set(hEnvT,  'XData', bx_r, 'YData', by_r);
    set(hDirT,  'XData', S.x, 'YData', S.y, ...
                'UData', vBx/v_len*v_disp, 'VData', vBy/v_len*v_disp);
    if length(S.trail_x)>1
        set(hTrailT,'XData',S.trail_x,'YData',S.trail_y);
    end
    set(hLblT, 'Position', [S.x, S.y+dr_w+0.3]);

    % ── SIDE VIEW — horizontal oval profile ────────────────────────────────────
    t40 = linspace(0,2*pi,40);
    pv  = atan2(S.w, max(0.05, sqrt(S.u^2+S.v^2))) * 0.35;
    ex  = S.x + dr_f*cos(t40)*cos(pv) - dr_w*sin(t40)*sin(pv);
    ez  = S.z + dr_f*cos(t40)*sin(pv) + dr_w*sin(t40)*cos(pv);
    set(hEnvS, 'XData', ex, 'YData', ez);
    set(hGndS, 'XData', [S.x-0.4 S.x+0.4 S.x+0.4 S.x-0.4], ...
               'YData', [S.z-dr_w-0.12 S.z-dr_w-0.12 S.z-dr_w S.z-dr_w]);
    set(hDirS, 'XData', S.x, 'YData', S.z, ...
               'UData', cos(pv)*0.8, 'VData', sin(pv)*0.8);
    if length(S.trail_x)>1
        set(hTrailS,'XData',S.trail_x,'YData',S.trail_z);
    end

    % ── FRONT CAMERA VIEW — circular cross-section ─────────────────────────────
    set(hEnvF, 'XData', S.y+dr_w*cos(t40), 'YData', S.z+dr_w*sin(t40));
    if length(S.trail_y)>1
        set(hTrailF,'XData',S.trail_y,'YData',S.trail_z);
    end

    % ── 3D VIEW — vectorized rotation, low-poly ellipsoid ─────────────────────
    % Vectorized yaw rotation: no loop needed since heading is yaw only
    Xr = E0.x*cp - E0.y*sp + S.x;
    Yr = E0.x*sp + E0.y*cp + S.y;
    Zr = E0.z + S.z;
    set(hSurf, 'XData', Xr, 'YData', Yr, 'ZData', Zr);

    % Gondola — simplified box
    gd=0.28; gw=0.18; gh=0.10;
    cb=[-gd/2 -gw/2 -gh; gd/2 -gw/2 -gh; gd/2 gw/2 -gh; -gd/2 gw/2 -gh; ...
        -gd/2 -gw/2  0;  gd/2 -gw/2  0;  gd/2 gw/2  0;  -gd/2 gw/2  0];
    R2=[cp -sp 0;sp cp 0;0 0 1];
    cw=(R2*cb')'; cw(:,1)=cw(:,1)+S.x; cw(:,2)=cw(:,2)+S.y; cw(:,3)=cw(:,3)+S.z-dr_w;
    set(hGond3,'Vertices',cw,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;3 4 8 7;1 4 8 5;2 3 7 6]);

    % Fins — simplified triangles
    a3=P.a; b3=P.b;
    fL=[[-a3*.85 0 0];[-a3*1.08  b3*.85 0];[-a3*.65  b3*.35 0]];
    fR=[[-a3*.85 0 0];[-a3*1.08 -b3*.85 0];[-a3*.65 -b3*.35 0]];
    fT=[[-a3*.85 0 0];[-a3*1.08 0  b3*.85];[-a3*.65 0  b3*.35]];
    for ii=1:3
        fL(ii,:)=(R2*fL(ii,:)')'+[S.x S.y S.z];
        fR(ii,:)=(R2*fR(ii,:)')'+[S.x S.y S.z];
        fT(ii,:)=(R2*fT(ii,:)')'+[S.x S.y S.z];
    end
    set(hFin3L,'XData',fL(:,1),'YData',fL(:,2),'ZData',fL(:,3));
    set(hFin3R,'XData',fR(:,1),'YData',fR(:,2),'ZData',fR(:,3));
    set(hFin3T,'XData',fT(:,1),'YData',fT(:,2),'ZData',fT(:,3));
    set(hDir3,'XData',S.x,'YData',S.y,'ZData',S.z, ...
              'UData',cp*1.2,'VData',sp*1.2,'WData',0);
    if length(S.trail_x)>1
        set(hTrail3,'XData',S.trail_x,'YData',S.trail_y,'ZData',S.trail_z);
    end
end
