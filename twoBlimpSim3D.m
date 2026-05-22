function twoBlimpSim3D()
%% blimpSim3D  — 4-panel interactive blimp simulation
% Panels: Side (X-Z) | Top (X-Y) | Front camera (Y-Z) | 3D perspective
%
% CONTROLS (click figure first):
%   Up/Down    arrows  — Motor 3: rise / sink
%   Left/Right arrows  — Motors 1+2: yaw + surge
%   R  — Reset        Q or Esc — Quit

%% ── Parameters (Sponaugle 2025 Thesis) ─────────────────────────────────────
P.m_surge = 0.460 + 0.1091;   % effective surge mass [kg]
P.m_heave = 0.460 + 0.3120;   % effective heave mass [kg]
P.Iz      = 0.0873 + 0.0197;  % effective yaw inertia [kg*m^2]
P.Xu      = 0.1900;            % surge damping [kg/s]
P.Zw      = 0.3366;            % heave damping [kg/s]
P.Nr      = 0.2450;            % yaw damping [kg*m^2/s]
P.ly      = 0.461;             % lateral motor moment arm [m]
P.F_max   = 1.07;              % motor saturation [N]
P.F_step  = 0.40;              % lateral force per key [N]
P.F_vert  = 0.55;              % vertical force per key [N]
P.a       = 0.55;              % envelope semi-axis X (length) [m]
P.b       = 0.22;              % envelope semi-axis Y (width)  [m]
P.c       = 0.22;              % envelope semi-axis Z (height) [m]
P.capture_r = 0.35;            % capture radius [m] — get within this to score
% Inter-agent communication distance zones
P.comm_min_red    = 0.40;      % [m] danger-close: collision risk
P.comm_min_yellow = 0.80;      % [m] too close: approaching danger
P.comm_ideal_lo   = 0.80;      % [m] ideal range lower bound
P.comm_ideal_hi   = 2.50;      % [m] ideal range upper bound
P.comm_max_yellow = 3.20;      % [m] too far: approaching comm loss
P.comm_max_red    = 4.00;      % [m] comm lost: out of range

%% ── Pre-compute unit ellipsoid mesh (body frame, origin-centred) ────────────
N = 20;
[us, vs] = meshgrid(linspace(0, 2*pi, N), linspace(0, pi, N));
E0.x = P.a .* cos(us) .* sin(vs);
E0.y = P.b .* sin(us) .* sin(vs);
E0.z = P.c .* cos(vs);
E0.n = N;

%% ── Colours ─────────────────────────────────────────────────────────────────
BG  = [0.04 0.08 0.05];
BG2 = [0.02 0.05 0.03];
GRN = [0.17 1.00 0.56];
DGR = [0.10 0.40 0.22];
ORG = [1.00 0.60 0.10];

%% ── Figure ───────────────────────────────────────────────────────────────────
fig = figure('Name','WVU Airship  3D Simulation', ...
    'NumberTitle','off', 'Color',BG, 'Position',[40 40 1280 720], ...
    'KeyPressFcn',     @onKeyDown, ...
    'KeyReleaseFcn',   @onKeyUp, ...
    'CloseRequestFcn', @onClose, ...
    'Toolbar','none', 'Menubar','none');

%% ── Helper: build styled 2D axes ─────────────────────────────────────────────
    function ax = ax2D(pos, xl, yl, ttl)
        ax = axes('Parent',fig, 'Units','pixels', 'Position',pos, ...
            'Color',BG2, 'XColor',DGR, 'YColor',DGR, ...
            'GridColor',[0.07 0.20 0.11], 'GridAlpha',1, ...
            'XGrid','on', 'YGrid','on', 'Box','on', ...
            'FontName','Courier New', 'FontSize',8);
        hold(ax,'on');  axis(ax,'manual');
        title(ax, ttl, 'Color',GRN, 'FontName','Courier New', 'FontSize',9);
        xlabel(ax, xl, 'Color',DGR);
        ylabel(ax, yl, 'Color',DGR);
    end

axS = ax2D([18  375 335 310], 'X [m]', 'Z [m]',  'SIDE  (X - Z)');
xlim(axS,[-4 4]);  ylim(axS,[-1.5 3.5]);

axT = ax2D([365 375 335 310], 'X [m]', 'Y [m]',  'TOP  (X - Y)');
xlim(axT,[-4 4]);  ylim(axT,[-4 4]);

axF = ax2D([18   45 335 300], 'Y [m]', 'Z [m]',  'FRONT CAMERA  (Y - Z)');
xlim(axF,[-3 3]);  ylim(axF,[-1 3]);

ax3 = axes('Parent',fig, 'Units','pixels', 'Position',[365 45 500 300], ...
    'Color',[0.02 0.04 0.03], 'XColor',DGR, 'YColor',DGR, 'ZColor',DGR, ...
    'GridColor',[0.07 0.20 0.11], 'GridAlpha',0.8, ...
    'XGrid','on', 'YGrid','on', 'ZGrid','on', 'Box','on', ...
    'FontName','Courier New', 'FontSize',7, 'Projection','perspective');
hold(ax3,'on');  view(ax3, 38, 22);
xlim(ax3,[-3 3]);  ylim(ax3,[-3 3]);  zlim(ax3,[-1 3]);
title(ax3,'3D PERSPECTIVE', 'Color',GRN, 'FontName','Courier New', 'FontSize',9);
xlabel(ax3,'X','Color',DGR);  ylabel(ax3,'Y','Color',DGR);  zlabel(ax3,'Z','Color',DGR);

%% ── State panel ──────────────────────────────────────────────────────────────
axP = axes('Parent',fig, 'Units','pixels', 'Position',[878 45 192 640], ...
    'Color',BG2, 'Visible','off');
hold(axP,'on');  xlim(axP,[0 1]);  ylim(axP,[0 1]);
text(axP,0.5,0.97,'VEHICLE STATE','Color',GRN,'FontName','Courier New', ...
    'FontSize',9,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');

sNames = {'x','y','z','psi','u','w','r'};
sUnits = {'m','m','m','deg','m/s','m/s','r/s'};
sv = gobjects(7,1);
for i = 1:7
    yp = 0.89 - (i-1)*0.107;
    text(axP, 0.05, yp+0.048, sprintf('%s  [%s]', upper(sNames{i}), sUnits{i}), ...
        'Color',DGR,'FontName','Courier New','FontSize',7,'Units','normalized');
    sv(i) = text(axP, 0.05, yp, '0.000', 'Color',GRN, ...
        'FontName','Courier New','FontSize',13,'FontWeight','bold','Units','normalized');
end

mCol = {[0.13 0.75 0.42],[0.13 0.75 0.42],[0.15 0.55 0.95]};
mb   = gobjects(3,1);
mLbl = {'M1 LEFT','M2 RIGHT','M3 VERT'};
for i = 1:3
    yb = 0.01 + (i-1)*0.055;
    patch(axP,[0.05 0.95 0.95 0.05],[yb yb yb+0.04 yb+0.04], ...
        [0.04 0.10 0.06],'EdgeColor',[0.08 0.22 0.12]);
    mb(i) = patch(axP,[0.05 0.06 0.06 0.05],[yb yb yb+0.04 yb+0.04], ...
        mCol{i},'EdgeColor','none');
    text(axP,0.05,yb+0.042,mLbl{i},'Color',DGR,'FontName','Courier New', ...
        'FontSize',6,'Units','normalized');
end

text(axP,0.5,0.225,'SCORE','Color',DGR,'FontName','Courier New',...
    'FontSize',7,'HorizontalAlignment','center','Units','normalized');
svScore = text(axP,0.5,0.175,'0','Color',[0.17 1.00 0.56],...
    'FontName','Courier New','FontSize',22,'FontWeight','bold',...
    'HorizontalAlignment','center','Units','normalized');
text(axP,0.5,0.135,'DIST TO TARGET','Color',DGR,'FontName','Courier New',...
    'FontSize',7,'HorizontalAlignment','center','Units','normalized');
svDist = text(axP,0.5,0.09,'-.-- m','Color',[0.17 1.00 0.56],...
    'FontName','Courier New','FontSize',12,'FontWeight','bold',...
    'HorizontalAlignment','center','Units','normalized');
svBear = text(axP,0.5,0.055,'BRG --.-','Color',DGR,'FontName','Courier New',...
    'FontSize',8,'HorizontalAlignment','center','Units','normalized');

annotation(fig,'textbox',[0 0 1 0.04], ...
    'String','  A: Arrows=yaw+altitude     B: WASD=yaw+altitude     R=Reset  N=New target  Q=Quit', ...
    'Color',[0.2 0.6 0.3],'BackgroundColor',[0.02 0.06 0.03], ...
    'EdgeColor',[0.07 0.25 0.12],'FontName','Courier New','FontSize',9, ...
    'VerticalAlignment','middle');

%% ── Graphic objects ──────────────────────────────────────────────────────────
% Side view
hTS   = plot(axS, NaN,NaN, '-', 'Color',[0.1 0.55 0.3], 'LineWidth',1.1);
hEnvS = patch(axS,'XData',0,'YData',0,'FaceColor',[0.07 0.25 0.13], ...
    'EdgeColor',GRN,'LineWidth',1.2);
hGndS = patch(axS,'XData',0,'YData',0,'FaceColor',[0.04 0.13 0.08], ...
    'EdgeColor',[0.1 0.35 0.18],'LineWidth',0.8);
hDirS = quiver(axS,0,0,0.5,0,0,'Color',GRN,'LineWidth',2,'MaxHeadSize',0.8,'AutoScale','off');
hVelS = quiver(axS,0,0,0,0,  0,'Color',ORG,'LineWidth',1.5,'MaxHeadSize',0.6,'AutoScale','off');
plot(axS,[-10 10],[-1.5 -1.5],'--','Color',[0.45 0.12 0.12],'LineWidth',0.8);
text(axS,-3.7,-1.7,'GROUND','Color',[0.45 0.12 0.12],'FontName','Courier New','FontSize',7);

% Top view
hTT   = plot(axT, NaN,NaN, '-', 'Color',[0.1 0.55 0.3], 'LineWidth',1.1);
hEnvT = patch(axT,'XData',0,'YData',0,'FaceColor',[0.07 0.25 0.13], ...
    'EdgeColor',GRN,'LineWidth',1.2);
hDirT = quiver(axT,0,0,0.5,0,0,'Color',GRN,'LineWidth',2,'MaxHeadSize',0.8,'AutoScale','off');
hVelT = quiver(axT,0,0,0,0,  0,'Color',ORG,'LineWidth',1.5,'MaxHeadSize',0.6,'AutoScale','off');
plot(axT,0,0,'+','Color',DGR,'MarkerSize',10,'LineWidth',1.5);

% Front camera
th_c = linspace(0,2*pi,40);
hTF   = plot(axF, NaN,NaN, '-', 'Color',[0.1 0.55 0.3], 'LineWidth',1.1);
hEnvF = patch(axF,'XData',0,'YData',0,'FaceColor',[0.07 0.25 0.13], ...
    'EdgeColor',GRN,'LineWidth',1.3,'FaceAlpha',0.75);
hGndF = patch(axF,'XData',0,'YData',0,'FaceColor',[0.04 0.13 0.08], ...
    'EdgeColor',[0.1 0.35 0.18],'LineWidth',0.8);
plot(axF,[-5 5],[0 0],'--','Color',[0.45 0.12 0.12],'LineWidth',0.8);
text(axF,-2.85,-0.15,'HORIZON','Color',[0.45 0.12 0.12],'FontName','Courier New','FontSize',7);
plot(axF,0,0,'+','Color',DGR,'MarkerSize',16,'LineWidth',1);
plot(axF, 0.12*cos(th_c), 0.12*sin(th_c)+1,'Color',GRN,'LineWidth',0.8);
text(axF,0.15,0.94,'NOSE','Color',GRN,'FontName','Courier New','FontSize',7);

% 3D surface
hSurf = surf(ax3,E0.x,E0.y,E0.z,'FaceColor',[0.08 0.28 0.14], ...
    'EdgeColor',GRN,'FaceAlpha',0.82,'EdgeAlpha',0.25,'LineWidth',0.3);
light(ax3,'Position',[ 2 -2  3],'Style','infinite');
light(ax3,'Position',[-1  1 -1],'Style','infinite','Color',[0.05 0.15 0.08]);
lighting(ax3,'gouraud');  material(ax3,'dull');

hGond3  = patch(ax3,'Vertices',zeros(8,3),'Faces',ones(6,4), ...
    'FaceColor',[0.05 0.15 0.08],'EdgeColor',[0.15 0.5 0.25], ...
    'LineWidth',0.8,'FaceAlpha',0.9);
hFin3L  = patch(ax3,'XData',0,'YData',0,'ZData',0, ...
    'FaceColor',[0.06 0.20 0.11],'EdgeColor',GRN,'EdgeAlpha',0.55,'LineWidth',0.9,'FaceAlpha',0.75);
hFin3R  = patch(ax3,'XData',0,'YData',0,'ZData',0, ...
    'FaceColor',[0.06 0.20 0.11],'EdgeColor',GRN,'EdgeAlpha',0.55,'LineWidth',0.9,'FaceAlpha',0.75);
hFin3T  = patch(ax3,'XData',0,'YData',0,'ZData',0, ...
    'FaceColor',[0.06 0.20 0.11],'EdgeColor',GRN,'EdgeAlpha',0.55,'LineWidth',0.9,'FaceAlpha',0.75);
hTrail3 = plot3(ax3,NaN,NaN,NaN,'-','Color',[0.1 0.55 0.3],'LineWidth',1.3);
hDir3   = quiver3(ax3,0,0,0,0.65,0,0,0,'Color',GRN,'LineWidth',2, ...
    'MaxHeadSize',0.5,'AutoScale','off');
hShadow = plot3(ax3,NaN,NaN,NaN,'--','Color',[0.15 0.45 0.22],'LineWidth',0.8);

[gxf,gyf] = meshgrid(-4:1:4,-4:1:4);
surf(ax3,gxf,gyf,-ones(size(gxf)),'FaceColor','none', ...
    'EdgeColor',[0.06 0.18 0.10],'EdgeAlpha',0.5,'LineWidth',0.4);


%% ── Target markers ──────────────────────────────────────────────────────────
TGT_CLR = [1.00 0.85 0.10];   % gold
TGT_CLR2= [1.00 0.50 0.05];   % orange ring

% Side view: vertical line + diamond
hTgtLineS = plot(axS,[NaN NaN],[NaN NaN],'--','Color',TGT_CLR,'LineWidth',1);
hTgtMarkS = plot(axS,NaN,NaN,'d','Color',TGT_CLR,'MarkerFaceColor',TGT_CLR,...
    'MarkerSize',10,'LineWidth',1.5);

% Top view: crosshair + diamond
hTgtLineT = plot(axT,[NaN NaN],[NaN NaN],'--','Color',TGT_CLR,'LineWidth',1);
hTgtMarkT = plot(axT,NaN,NaN,'d','Color',TGT_CLR,'MarkerFaceColor',TGT_CLR,...
    'MarkerSize',10,'LineWidth',1.5);

% Front view: diamond + vertical drop line
hTgtLineF = plot(axF,[NaN NaN],[NaN NaN],'--','Color',TGT_CLR,'LineWidth',1);
hTgtMarkF = plot(axF,NaN,NaN,'d','Color',TGT_CLR,'MarkerFaceColor',TGT_CLR,...
    'MarkerSize',10,'LineWidth',1.5);

% Capture ring (drawn around blimp when close, on top view)
th_cap = linspace(0,2*pi,60);
hCapRing = plot(axT, NaN, NaN, '-', 'Color',TGT_CLR2,'LineWidth',1.5);

% 3D target sphere (pre-built unit sphere scaled to capture radius)
[tsx,tsy,tsz] = sphere(18);
r_cap = P.capture_r;
hTgt3  = surf(ax3, tsx*r_cap, tsy*r_cap, tsz*r_cap, ...
    'FaceColor',TGT_CLR,'EdgeColor','none','FaceAlpha',0.35);
hTgtRing3 = plot3(ax3, r_cap*cos(th_cap), r_cap*sin(th_cap), zeros(size(th_cap)),...
    '--','Color',TGT_CLR2,'LineWidth',1.2);
% Vertical drop line in 3D
hTgtDrop3 = plot3(ax3,[NaN NaN],[NaN NaN],[NaN NaN],'--','Color',TGT_CLR,...
    'LineWidth',0.8);
% Floor shadow of target
hTgtFloor3= plot3(ax3,NaN,NaN,NaN,'+','Color',TGT_CLR,'MarkerSize',14,'LineWidth',2);


%% ── Agent B (WASD) graphic objects ─────────────────────────────────────────
CYN  = [0.10 0.85 0.95];   % cyan — Agent B colour
CYN2 = [0.05 0.55 0.70];

% Side view
hB_EnvS = patch(axS,'XData',0,'YData',0,'FaceColor',[0.04 0.25 0.35], ...
    'EdgeColor',CYN,'LineWidth',1.2);
hB_GndS = patch(axS,'XData',0,'YData',0,'FaceColor',[0.03 0.15 0.22], ...
    'EdgeColor',CYN2,'LineWidth',0.7);
hB_DirS = quiver(axS,0,0,0.5,0,0,'Color',CYN,'LineWidth',1.6,'MaxHeadSize',0.8,'AutoScale','off');
hB_TS   = plot(axS,NaN,NaN,'-','Color',CYN2,'LineWidth',0.9);

% Top view
hB_EnvT = patch(axT,'XData',0,'YData',0,'FaceColor',[0.04 0.25 0.35], ...
    'EdgeColor',CYN,'LineWidth',1.2);
hB_DirT = quiver(axT,0,0,0.5,0,0,'Color',CYN,'LineWidth',1.6,'MaxHeadSize',0.8,'AutoScale','off');
hB_TT   = plot(axT,NaN,NaN,'-','Color',CYN2,'LineWidth',0.9);

% Front camera
hB_EnvF = patch(axF,'XData',0,'YData',0,'FaceColor',[0.04 0.25 0.35], ...
    'EdgeColor',CYN,'LineWidth',1.2,'FaceAlpha',0.65);
hB_TF   = plot(axF,NaN,NaN,'-','Color',CYN2,'LineWidth',0.9);

% 3D surface
hB_Surf  = surf(ax3,E0.x,E0.y,E0.z+2, ...
    'FaceColor',[0.05 0.28 0.38],'EdgeColor',CYN,'FaceAlpha',0.78,'EdgeAlpha',0.2,'LineWidth',0.3);
hB_Gond3 = patch(ax3,'Vertices',zeros(8,3),'Faces',ones(6,4), ...
    'FaceColor',[0.03 0.15 0.22],'EdgeColor',CYN2,'LineWidth',0.8,'FaceAlpha',0.9);
hB_Fin3L = patch(ax3,'XData',0,'YData',0,'ZData',0, ...
    'FaceColor',[0.04 0.20 0.28],'EdgeColor',CYN,'EdgeAlpha',0.6,'LineWidth',0.8,'FaceAlpha',0.7);
hB_Fin3R = patch(ax3,'XData',0,'YData',0,'ZData',0, ...
    'FaceColor',[0.04 0.20 0.28],'EdgeColor',CYN,'EdgeAlpha',0.6,'LineWidth',0.8,'FaceAlpha',0.7);
hB_Fin3T = patch(ax3,'XData',0,'YData',0,'ZData',0, ...
    'FaceColor',[0.04 0.20 0.28],'EdgeColor',CYN,'EdgeAlpha',0.6,'LineWidth',0.8,'FaceAlpha',0.7);
hB_Dir3  = quiver3(ax3,0,2,0,0.65,0,0,0,'Color',CYN,'LineWidth',1.8,'MaxHeadSize',0.5,'AutoScale','off');
hB_Trail3= plot3(ax3,NaN,NaN,NaN,'-','Color',CYN2,'LineWidth',1.0);
light(ax3,'Position',[0 3 2],'Style','infinite','Color',[0.04 0.18 0.25]);

% Inter-agent communication line (colour-coded by zone)
hLink_T  = plot(axT, NaN,NaN,'-','Color',[0.3 0.9 0.3],'LineWidth',1.8);
hLink_3  = plot3(ax3,NaN,NaN,NaN,'-','Color',[0.3 0.9 0.3],'LineWidth',1.8);

% Communication boundary rings centred on Agent A (top view)
th_comm = linspace(0,2*pi,80);
hCommInner_T = plot(axT, NaN,NaN,'-','Color',[1 0.3 0.3],'LineWidth',1.2);  % danger-close
hCommLo_T    = plot(axT, NaN,NaN,'--','Color',[1 0.85 0.1],'LineWidth',1.0); % min-yellow
hCommHi_T    = plot(axT, NaN,NaN,'--','Color',[1 0.85 0.1],'LineWidth',1.0); % max-yellow
hCommOuter_T = plot(axT, NaN,NaN,'-','Color',[1 0.3 0.3],'LineWidth',1.2);  % comm-lost

% Same rings in 3D (horizontal circles at mean altitude)
hCommOuter_3 = plot3(ax3,NaN,NaN,NaN,'-','Color',[1 0.3 0.3],'LineWidth',1.0);
hCommHi_3    = plot3(ax3,NaN,NaN,NaN,'--','Color',[1 0.85 0.1],'LineWidth',0.8);

% Agent B label on top view
hB_LblT = text(axT, 0, 2.0, 'B','Color',CYN,'FontName','Courier New', ...
    'FontSize',8,'FontWeight','bold','HorizontalAlignment','center');
hA_LblT = text(axT, 0, 0,   'A','Color',[0.17 1.00 0.56],'FontName','Courier New', ...
    'FontSize',8,'FontWeight','bold','HorizontalAlignment','center');

% Inter-agent distance + comm zone panel
text(axP,0.5,0.375,'COMM LINK','Color',[0.5 0.5 0.3],'FontName','Courier New', ...
    'FontSize',7,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');
svABDist = text(axP,0.5,0.325,'-.-- m','Color',[0.3 0.9 0.3], ...
    'FontName','Courier New','FontSize',13,'FontWeight','bold', ...
    'HorizontalAlignment','center','Units','normalized');
svCommZone = text(axP,0.5,0.275,'IDEAL','Color',[0.3 0.9 0.3], ...
    'FontName','Courier New','FontSize',8,'FontWeight','bold', ...
    'HorizontalAlignment','center','Units','normalized');
% Zone bar: 5 coloured blocks representing the 5 distance zones
% [RED|YELLOW|GREEN|YELLOW|RED]  with a pointer updated live
zone_y0 = 0.225; zone_h = 0.03;
zone_colors = {[0.6 0.1 0.1],[0.6 0.5 0.05],[0.1 0.5 0.15],[0.6 0.5 0.05],[0.6 0.1 0.1]};
zone_w = 0.18;
for zi=1:5
    patch(axP,[0.05+(zi-1)*zone_w 0.05+zi*zone_w 0.05+zi*zone_w 0.05+(zi-1)*zone_w], ...
        [zone_y0 zone_y0 zone_y0+zone_h zone_y0+zone_h], ...
        zone_colors{zi},'EdgeColor',[0.02 0.05 0.03],'LineWidth',0.5);
end
% Zone labels
zone_lbls = {'CRIT','WARN','OK','WARN','CRIT'};
zone_xs   = 0.05 + (0:4)*zone_w + zone_w/2;
for zi=1:5
    text(axP,zone_xs(zi),zone_y0+zone_h/2+0.005,zone_lbls{zi}, ...
        'Color',[0.8 0.8 0.8],'FontName','Courier New','FontSize',5, ...
        'HorizontalAlignment','center','Units','normalized');
end
% Live pointer (triangle marker on zone bar)
svZonePtr = plot(axP, 0.5, zone_y0-0.008, '^', 'Color',[0.3 0.9 0.3], ...
    'MarkerFaceColor',[0.3 0.9 0.3],'MarkerSize',6);

%% ── Bundle handles ───────────────────────────────────────────────────────────
H.axS=axS; H.axT=axT; H.axF=axF; H.ax3=ax3; H.axP=axP;
H.hTS=hTS; H.hEnvS=hEnvS; H.hGndS=hGndS; H.hDirS=hDirS; H.hVelS=hVelS;
H.hTT=hTT; H.hEnvT=hEnvT; H.hDirT=hDirT; H.hVelT=hVelT;
H.hTF=hTF; H.hEnvF=hEnvF; H.hGndF=hGndF;
H.hSurf=hSurf; H.hGond3=hGond3; H.hTrail3=hTrail3;
H.hDir3=hDir3; H.hShadow=hShadow;
H.hFin3L=hFin3L; H.hFin3R=hFin3R; H.hFin3T=hFin3T;
H.sv=sv; H.mb=mb; H.svScore=svScore; H.svDist=svDist; H.svBear=svBear;
% Agent B
H.hB_EnvS=hB_EnvS; H.hB_GndS=hB_GndS; H.hB_DirS=hB_DirS; H.hB_TS=hB_TS;
H.hB_EnvT=hB_EnvT; H.hB_DirT=hB_DirT; H.hB_TT=hB_TT;
H.hB_EnvF=hB_EnvF; H.hB_TF=hB_TF;
H.hB_Surf=hB_Surf; H.hB_Gond3=hB_Gond3;
H.hB_Fin3L=hB_Fin3L; H.hB_Fin3R=hB_Fin3R; H.hB_Fin3T=hB_Fin3T;
H.hB_Dir3=hB_Dir3; H.hB_Trail3=hB_Trail3;
H.hLink_T=hLink_T; H.hLink_3=hLink_3;
H.hCommInner_T=hCommInner_T; H.hCommLo_T=hCommLo_T;
H.hCommHi_T=hCommHi_T; H.hCommOuter_T=hCommOuter_T;
H.hCommOuter_3=hCommOuter_3; H.hCommHi_3=hCommHi_3;
H.hB_LblT=hB_LblT; H.hA_LblT=hA_LblT;
H.svABDist=svABDist; H.svCommZone=svCommZone; H.svZonePtr=svZonePtr;
H.hTgtLineS=hTgtLineS; H.hTgtMarkS=hTgtMarkS;
H.hTgtLineT=hTgtLineT; H.hTgtMarkT=hTgtMarkT;
H.hTgtLineF=hTgtLineF; H.hTgtMarkF=hTgtMarkF;
H.hCapRing=hCapRing;
H.hTgt3=hTgt3; H.hTgtRing3=hTgtRing3; H.hTgtDrop3=hTgtDrop3; H.hTgtFloor3=hTgtFloor3;

%% ── Store & start timer ──────────────────────────────────────────────────────
S0 = initState();
S0.score = 0;
setappdata(fig,'S',  S0);
setappdata(fig,'SB', initStateB());
setappdata(fig,'T',  newTarget());
setappdata(fig,'P',  P);
setappdata(fig,'H',  H);
setappdata(fig,'E0', E0);
setappdata(fig,'running', true);

tmr = timer('Name','BlimpTimer3D','Period',0.025, ...
    'ExecutionMode','fixedRate','TimerFcn',@(~,~)timerTick(fig));
setappdata(fig,'tmr',tmr);
start(tmr);
fprintf('blimpSim3D running.  Up/Down=altitude  Left/Right=yaw+surge  R=reset  Q=quit\n');

%% ── Nested callbacks ─────────────────────────────────────────────────────────
    function timerTick(f)
        if ~ishandle(f) || ~getappdata(f,'running')
            safeStop(f); return;
        end
        S_  = getappdata(f,'S');
        SB_ = getappdata(f,'SB');
        P_  = getappdata(f,'P');
        H_  = getappdata(f,'H');
        E0_ = getappdata(f,'E0');

        T_  = getappdata(f,'T');
        % Agent B physics
        inpB = calcInputs(SB_, P_);
        SB_  = rk4(SB_, inpB, P_, 0.025);
        SB_.t = SB_.t + 0.025; SB_.frame = SB_.frame + 1;
        if mod(SB_.frame,3)==0
            SB_.trail_x(end+1)=SB_.x; SB_.trail_y(end+1)=SB_.y; SB_.trail_z(end+1)=SB_.z;
            if length(SB_.trail_x)>300
                SB_.trail_x=SB_.trail_x(2:end);
                SB_.trail_y=SB_.trail_y(2:end);
                SB_.trail_z=SB_.trail_z(2:end);
            end
        end
        inp = calcInputs(S_, P_);
        S_  = rk4(S_, inp, P_, 0.025);
        % Check capture
        dist3 = sqrt((S_.x-T_.x)^2+(S_.y-T_.y)^2+(S_.z-T_.z)^2);
        if dist3 < P_.capture_r && ~S_.captured
            S_.score   = S_.score + 1;
            S_.captured= true;
            S_.flash   = 12;   % flash frames
            T_ = newTarget();
            setappdata(f,'T',T_);
        elseif dist3 >= P_.capture_r
            S_.captured = false;
        end
        if S_.flash > 0, S_.flash = S_.flash - 1; end
        S_.t     = S_.t + 0.025;
        S_.frame = S_.frame + 1;

        if mod(S_.frame, 3) == 0
            S_.trail_x(end+1) = S_.x;
            S_.trail_y(end+1) = S_.y;
            S_.trail_z(end+1) = S_.z;
            if length(S_.trail_x) > 300
                S_.trail_x = S_.trail_x(2:end);
                S_.trail_y = S_.trail_y(2:end);
                S_.trail_z = S_.trail_z(2:end);
            end
        end

        setappdata(f,'S',S_);
        setappdata(f,'SB',SB_);
        drawAll(H_, S_, inp, SB_, inpB, P_, E0_, T_);
        drawnow limitrate;
    end

    function onKeyDown(~, evt)
        if ~ishandle(fig), return; end
        S_  = getappdata(fig,'S');
        SB_ = getappdata(fig,'SB');
        switch evt.Key
            % ── Agent A: arrow keys ──
            case 'uparrow',    S_.keys.up    = 1;
            case 'downarrow',  S_.keys.down  = 1;
            case 'leftarrow',  S_.keys.left  = 1;
            case 'rightarrow', S_.keys.right = 1;
            % ── Agent B: WASD ──
            case 'w',          SB_.keys.up    = 1;
            case 's',          SB_.keys.down  = 1;
            case 'a',          SB_.keys.left  = 1;
            case 'd',          SB_.keys.right = 1;
            % ── System ──
            case 'r'
                S_ = initState(); S_.score=0;
                SB_= initStateB();
                setappdata(fig,'T', newTarget());
            case 'n',  setappdata(fig,'T', newTarget());
            case {'q','escape'}
                setappdata(fig,'running',false);
                safeStop(fig);
                return;
        end
        setappdata(fig,'S',S_);
        setappdata(fig,'SB',SB_);
    end

    function onKeyUp(~, evt)
        if ~ishandle(fig), return; end
        S_  = getappdata(fig,'S');
        SB_ = getappdata(fig,'SB');
        switch evt.Key
            case 'uparrow',    S_.keys.up    = 0;
            case 'downarrow',  S_.keys.down  = 0;
            case 'leftarrow',  S_.keys.left  = 0;
            case 'rightarrow', S_.keys.right = 0;
            case 'w',          SB_.keys.up    = 0;
            case 's',          SB_.keys.down  = 0;
            case 'a',          SB_.keys.left  = 0;
            case 'd',          SB_.keys.right = 0;
        end
        setappdata(fig,'S',S_);
        setappdata(fig,'SB',SB_);
    end

    function onClose(src,~)
        setappdata(src,'running',false);
        safeStop(src);
        delete(src);
    end

    function safeStop(f)
        if ~ishandle(f), return; end
        try
            t_ = getappdata(f,'tmr');
            if isvalid(t_), stop(t_); delete(t_); end
        catch
        end
    end

end  % ── end blimpSim3D ───────────────────────────────────────────────────────


%% ════════════════════════════════════════════════════════════════════════════
%%  LOCAL FUNCTIONS
%% ════════════════════════════════════════════════════════════════════════════

function S = initState()
    S.x=0; S.y=0; S.z=0; S.psi=0;
    S.u=0; S.w=0; S.r=0;
    S.keys = struct('up',0,'down',0,'left',0,'right',0);
    S.trail_x=[]; S.trail_y=[]; S.trail_z=[];
    S.frame=0; S.t=0; S.score=0; S.captured=false; S.flash=0;
end

function S = initStateB()
    % Agent B starts 2 m to the side so the two blimps are separated
    S.x=0; S.y=2.0; S.z=0; S.psi=0;
    S.u=0; S.w=0; S.r=0;
    S.keys = struct('up',0,'down',0,'left',0,'right',0);
    S.trail_x=[]; S.trail_y=[]; S.trail_z=[];
    S.frame=0; S.t=0;
end

function inp = calcInputs(S, P)
    F1 = S.keys.right * P.F_step;
    F2 = S.keys.left  * P.F_step;
    Fz = (S.keys.up - S.keys.down) * P.F_vert;
    inp.Fx = F1 + F2;
    inp.Fz = Fz;
    inp.Mz = (F1 - F2) * P.ly;
    inp.F1 = F1;  inp.F2 = F2;  inp.F3 = abs(Fz);
end

function S = rk4(S, inp, P, dt)
    function d = f(s, inp, P)
        d.du   = (inp.Fx - P.Xu * s.u) / P.m_surge;
        d.dw   = (inp.Fz - P.Zw * s.w) / P.m_heave;
        d.dr   = (inp.Mz - P.Nr * s.r) / P.Iz;
        d.dx   = s.u * cos(s.psi);
        d.dy   = s.u * sin(s.psi);
        d.dz   = s.w;
        d.dpsi = s.r;
    end
    k1 = f(S, inp, P);
    s2 = struct('x',S.x+dt/2*k1.dx,'y',S.y+dt/2*k1.dy,'z',S.z+dt/2*k1.dz, ...
        'psi',S.psi+dt/2*k1.dpsi,'u',S.u+dt/2*k1.du,'w',S.w+dt/2*k1.dw,'r',S.r+dt/2*k1.dr);
    k2 = f(s2, inp, P);
    s3 = struct('x',S.x+dt/2*k2.dx,'y',S.y+dt/2*k2.dy,'z',S.z+dt/2*k2.dz, ...
        'psi',S.psi+dt/2*k2.dpsi,'u',S.u+dt/2*k2.du,'w',S.w+dt/2*k2.dw,'r',S.r+dt/2*k2.dr);
    k3 = f(s3, inp, P);
    s4 = struct('x',S.x+dt*k3.dx,'y',S.y+dt*k3.dy,'z',S.z+dt*k3.dz, ...
        'psi',S.psi+dt*k3.dpsi,'u',S.u+dt*k3.du,'w',S.w+dt*k3.dw,'r',S.r+dt*k3.dr);
    k4 = f(s4, inp, P);
    S.x   = S.x   + dt/6*(k1.dx  +2*k2.dx  +2*k3.dx  +k4.dx);
    S.y   = S.y   + dt/6*(k1.dy  +2*k2.dy  +2*k3.dy  +k4.dy);
    S.z   = S.z   + dt/6*(k1.dz  +2*k2.dz  +2*k3.dz  +k4.dz);
    S.psi = S.psi + dt/6*(k1.dpsi+2*k2.dpsi+2*k3.dpsi+k4.dpsi);
    S.u   = S.u   + dt/6*(k1.du  +2*k2.du  +2*k3.du  +k4.du);
    S.w   = S.w   + dt/6*(k1.dw  +2*k2.dw  +2*k3.dw  +k4.dw);
    S.r   = S.r   + dt/6*(k1.dr  +2*k2.dr  +2*k3.dr  +k4.dr);
end

function drawAll(H, S, inp, SB, inpB, P, E0, T)
    GRN = [0.17 1.00 0.56];
    ORG = [1.00 0.60 0.10];
    a=P.a; b=P.b; c=P.c;
    pitch_v = atan2(S.w, max(0.05, abs(S.u))) * 0.35;

    %% Side view ────────────────────────────────────────────────
    t = linspace(0,2*pi,40);
    ex = S.x + a*cos(t)*cos(pitch_v) - c*sin(t)*sin(pitch_v);
    ez = S.z + a*cos(t)*sin(pitch_v) + c*sin(t)*cos(pitch_v);
    set(H.hEnvS,'XData',ex,'YData',ez);
    gw=0.16; gh=0.08;
    set(H.hGndS,'XData',[S.x-gw/2 S.x+gw/2 S.x+gw/2 S.x-gw/2], ...
        'YData',[S.z-c-gh S.z-c-gh S.z-c S.z-c]);
    set(H.hDirS,'XData',S.x,'YData',S.z,'UData',cos(pitch_v)*0.48,'VData',sin(pitch_v)*0.48);
    set(H.hVelS,'XData',S.x,'YData',S.z,'UData',S.u*0.28,'VData',S.w*0.28);
    if length(S.trail_x)>1, set(H.hTS,'XData',S.trail_x,'YData',S.trail_z); end
    xlim(H.axS, S.x+[-4 4]);  ylim(H.axS, S.z+[-2 3]);

    %% Top view ─────────────────────────────────────────────────
    cp=cos(S.psi); sp=sin(S.psi);
    bx=a*cos(t); by=b*sin(t);
    tx=S.x+bx*cp-by*sp;  ty=S.y+bx*sp+by*cp;
    set(H.hEnvT,'XData',tx,'YData',ty);
    set(H.hDirT,'XData',S.x,'YData',S.y,'UData',cp*0.48,'VData',sp*0.48);
    set(H.hVelT,'XData',S.x,'YData',S.y,'UData',S.u*cp*0.3,'VData',S.u*sp*0.3);
    if length(S.trail_x)>1, set(H.hTT,'XData',S.trail_x,'YData',S.trail_y); end
    xlim(H.axT, S.x+[-4 4]);  ylim(H.axT, S.y+[-4 4]);

    %% Front camera (Y-Z) ───────────────────────────────────────
    app_w = b*abs(cp) + a*abs(sp);
    ey_f = S.y + min(app_w,a)*cos(t);
    ez_f = S.z + c*sin(t);
    set(H.hEnvF,'XData',ey_f,'YData',ez_f);
    set(H.hGndF,'XData',[S.y-0.09 S.y+0.09 S.y+0.09 S.y-0.09], ...
        'YData',[S.z-c-0.08 S.z-c-0.08 S.z-c S.z-c]);
    if length(S.trail_y)>1, set(H.hTF,'XData',S.trail_y,'YData',S.trail_z); end
    xlim(H.axF, S.y+[-3 3]);  ylim(H.axF, S.z+[-1.5 2.5]);

    %% 3D perspective ────────────────────────────────────────────
    R = [cp -sp 0; sp cp 0; 0 0 1];
    N = E0.n;
    Xr=zeros(N,N); Yr=Xr; Zr=Xr;
    for row=1:N
        for col=1:N
            v=R*[E0.x(row,col);E0.y(row,col);E0.z(row,col)];
            Xr(row,col)=v(1)+S.x; Yr(row,col)=v(2)+S.y; Zr(row,col)=v(3)+S.z;
        end
    end
    set(H.hSurf,'XData',Xr,'YData',Yr,'ZData',Zr);

    % Gondola box
    gw3=0.14; gd3=0.22; gh3=0.09;
    cb = [-gd3/2 -gw3/2 -gh3; gd3/2 -gw3/2 -gh3; gd3/2  gw3/2 -gh3; -gd3/2  gw3/2 -gh3;
          -gd3/2 -gw3/2  0;   gd3/2 -gw3/2  0;   gd3/2  gw3/2  0;   -gd3/2  gw3/2  0];
    cw = (R*cb')';
    cw(:,1)=cw(:,1)+S.x; cw(:,2)=cw(:,2)+S.y; cw(:,3)=cw(:,3)+S.z-c;
    fg = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 1 4 8 5; 2 3 7 6];
    set(H.hGond3,'Vertices',cw,'Faces',fg);

    % Tail fins
    fL = [[-a*.85 0 0];[-a*1.08  b*.85 0];[-a*.65  b*.35 0]];
    fR = [[-a*.85 0 0];[-a*1.08 -b*.85 0];[-a*.65 -b*.35 0]];
    fT = [[-a*.85 0 0];[-a*1.08 0 c*.85]; [-a*.65 0 c*.35]];
    for ii=1:3
        fL(ii,:)=(R*fL(ii,:)')'+[S.x S.y S.z];
        fR(ii,:)=(R*fR(ii,:)')'+[S.x S.y S.z];
        fT(ii,:)=(R*fT(ii,:)')'+[S.x S.y S.z];
    end
    set(H.hFin3L,'XData',fL(:,1),'YData',fL(:,2),'ZData',fL(:,3));
    set(H.hFin3R,'XData',fR(:,1),'YData',fR(:,2),'ZData',fR(:,3));
    set(H.hFin3T,'XData',fT(:,1),'YData',fT(:,2),'ZData',fT(:,3));

    set(H.hDir3,'XData',S.x,'YData',S.y,'ZData',S.z,'UData',cp*0.65,'VData',sp*0.65,'WData',0);
    if length(S.trail_x)>1
        set(H.hTrail3,'XData',S.trail_x,'YData',S.trail_y,'ZData',S.trail_z);
        set(H.hShadow,'XData',S.trail_x,'YData',S.trail_y,'ZData',-ones(size(S.trail_x)));
    end
    xlim(H.ax3,S.x+[-3 3]); ylim(H.ax3,S.y+[-3 3]); zlim(H.ax3,S.z+[-1.5 2.5]);



    %% Agent B (WASD) ────────────────────────────────────────────
    CYN  = [0.10 0.85 0.95];
    CYN2 = [0.05 0.55 0.70];
    pitch_vB = atan2(SB.w, max(0.05, abs(SB.u))) * 0.35;
    cpB=cos(SB.psi); spB=sin(SB.psi);

    % Side view
    exB = SB.x + a*cos(t)*cos(pitch_vB) - c*sin(t)*sin(pitch_vB);
    ezB = SB.z + a*cos(t)*sin(pitch_vB) + c*sin(t)*cos(pitch_vB);
    set(H.hB_EnvS,'XData',exB,'YData',ezB);
    set(H.hB_GndS,'XData',[SB.x-gw/2 SB.x+gw/2 SB.x+gw/2 SB.x-gw/2], ...
        'YData',[SB.z-c-gh SB.z-c-gh SB.z-c SB.z-c]);
    set(H.hB_DirS,'XData',SB.x,'YData',SB.z, ...
        'UData',cos(pitch_vB)*0.48,'VData',sin(pitch_vB)*0.48);
    if length(SB.trail_x)>1, set(H.hB_TS,'XData',SB.trail_x,'YData',SB.trail_z); end

    % Top view
    bxB=a*cos(t); byB=b*sin(t);
    txB=SB.x+bxB*cpB-byB*spB; tyB=SB.y+bxB*spB+byB*cpB;
    set(H.hB_EnvT,'XData',txB,'YData',tyB);
    set(H.hB_DirT,'XData',SB.x,'YData',SB.y,'UData',cpB*0.48,'VData',spB*0.48);
    if length(SB.trail_x)>1, set(H.hB_TT,'XData',SB.trail_x,'YData',SB.trail_y); end
    set(H.hB_LblT,'Position',[SB.x SB.y+0.35]);
    set(H.hA_LblT,'Position',[S.x  S.y+0.35]);

    % Front camera
    app_wB = b*abs(cpB) + a*abs(spB);
    set(H.hB_EnvF,'XData',SB.y+min(app_wB,a)*cos(t),'YData',SB.z+c*sin(t));
    if length(SB.trail_y)>1, set(H.hB_TF,'XData',SB.trail_y,'YData',SB.trail_z); end

    % 3D
    RB = [cpB -spB 0; spB cpB 0; 0 0 1];
    Xrb=zeros(N,N); Yrb=Xrb; Zrb=Xrb;
    for row=1:N
        for col=1:N
            v=RB*[E0.x(row,col);E0.y(row,col);E0.z(row,col)];
            Xrb(row,col)=v(1)+SB.x; Yrb(row,col)=v(2)+SB.y; Zrb(row,col)=v(3)+SB.z;
        end
    end
    set(H.hB_Surf,'XData',Xrb,'YData',Yrb,'ZData',Zrb);

    cbB = [-gd3/2 -gw3/2 -gh3; gd3/2 -gw3/2 -gh3; gd3/2  gw3/2 -gh3; -gd3/2  gw3/2 -gh3;
           -gd3/2 -gw3/2  0;   gd3/2 -gw3/2  0;   gd3/2  gw3/2  0;   -gd3/2  gw3/2  0];
    cwB=(RB*cbB')'; cwB(:,1)=cwB(:,1)+SB.x; cwB(:,2)=cwB(:,2)+SB.y; cwB(:,3)=cwB(:,3)+SB.z-c;
    set(H.hB_Gond3,'Vertices',cwB,'Faces',fg);

    fBL=[[-a*.85 0 0];[-a*1.08  b*.85 0];[-a*.65  b*.35 0]];
    fBR=[[-a*.85 0 0];[-a*1.08 -b*.85 0];[-a*.65 -b*.35 0]];
    fBT=[[-a*.85 0 0];[-a*1.08 0 c*.85];[-a*.65 0 c*.35]];
    for ii=1:3
        fBL(ii,:)=(RB*fBL(ii,:)')'+[SB.x SB.y SB.z];
        fBR(ii,:)=(RB*fBR(ii,:)')'+[SB.x SB.y SB.z];
        fBT(ii,:)=(RB*fBT(ii,:)')'+[SB.x SB.y SB.z];
    end
    set(H.hB_Fin3L,'XData',fBL(:,1),'YData',fBL(:,2),'ZData',fBL(:,3));
    set(H.hB_Fin3R,'XData',fBR(:,1),'YData',fBR(:,2),'ZData',fBR(:,3));
    set(H.hB_Fin3T,'XData',fBT(:,1),'YData',fBT(:,2),'ZData',fBT(:,3));
    set(H.hB_Dir3,'XData',SB.x,'YData',SB.y,'ZData',SB.z,'UData',cpB*0.65,'VData',spB*0.65,'WData',0);
    if length(SB.trail_x)>1
        set(H.hB_Trail3,'XData',SB.trail_x,'YData',SB.trail_y,'ZData',SB.trail_z);
    end

    % Inter-agent link line (top view + 3D)

    % ── Inter-agent communication zones ─────────────────────────────────
    ab_dist = sqrt((S.x-SB.x)^2+(S.y-SB.y)^2+(S.z-SB.z)^2);
    ab_horiz= sqrt((S.x-SB.x)^2+(S.y-SB.y)^2);   % horizontal separation
    th_c    = linspace(0,2*pi,80);

    % Determine zone and colours
    if ab_dist < P.comm_min_red
        zoneStr = 'DANGER CLOSE';  abCol=[1.0 0.20 0.20]; linkStyle='-'; linkW=2.5;
    elseif ab_dist < P.comm_min_yellow
        zoneStr = 'TOO CLOSE';     abCol=[1.0 0.80 0.10]; linkStyle='--'; linkW=1.8;
    elseif ab_dist <= P.comm_ideal_hi
        zoneStr = 'IDEAL';         abCol=[0.20 1.0 0.40]; linkStyle='-';  linkW=1.8;
    elseif ab_dist < P.comm_max_yellow
        zoneStr = 'TOO FAR';       abCol=[1.0 0.80 0.10]; linkStyle='--'; linkW=1.8;
    else
        zoneStr = 'COMM LOST';     abCol=[1.0 0.20 0.20]; linkStyle=':';  linkW=1.2;
    end

    % Update comm link line style and colour
    set(H.hLink_T,'XData',[S.x SB.x],'YData',[S.y SB.y], ...
        'Color',abCol,'LineStyle',linkStyle,'LineWidth',linkW);
    set(H.hLink_3,'XData',[S.x SB.x],'YData',[S.y SB.y],'ZData',[S.z SB.z], ...
        'Color',abCol,'LineStyle',linkStyle,'LineWidth',linkW);

    % Communication boundary rings (centred on A, top view)
    z_mid = (S.z + SB.z) / 2;
    set(H.hCommInner_T,'XData',S.x+P.comm_min_red*cos(th_c), ...
        'YData',S.y+P.comm_min_red*sin(th_c));
    set(H.hCommLo_T,'XData',S.x+P.comm_min_yellow*cos(th_c), ...
        'YData',S.y+P.comm_min_yellow*sin(th_c));
    set(H.hCommHi_T,'XData',S.x+P.comm_max_yellow*cos(th_c), ...
        'YData',S.y+P.comm_max_yellow*sin(th_c));
    set(H.hCommOuter_T,'XData',S.x+P.comm_max_red*cos(th_c), ...
        'YData',S.y+P.comm_max_red*sin(th_c));

    % Outer rings in 3D at midpoint altitude
    set(H.hCommOuter_3,'XData',S.x+P.comm_max_red*cos(th_c), ...
        'YData',S.y+P.comm_max_red*sin(th_c), ...
        'ZData',z_mid*ones(1,80));
    set(H.hCommHi_3,'XData',S.x+P.comm_max_yellow*cos(th_c), ...
        'YData',S.y+P.comm_max_yellow*sin(th_c), ...
        'ZData',z_mid*ones(1,80));

    % Zone pointer on bar (map ab_dist → x position across 5 zones)
    % Bar spans x=0.05..0.95 in normalised units, zones:
    %  [0  comm_min_red  comm_min_yellow  comm_ideal_hi  comm_max_yellow  comm_max_red  inf]
    zone_edges = [0, P.comm_min_red, P.comm_min_yellow, ...
                  P.comm_ideal_hi, P.comm_max_yellow, P.comm_max_red];
    d_clamped = min(ab_dist, P.comm_max_red);
    % Piecewise linear mapping into [0.05, 0.95]
    seg = find(d_clamped >= zone_edges, 1, 'last');
    seg = min(seg, 5);
    seg_lo = zone_edges(seg);  seg_hi = zone_edges(seg+1);
    frac = (d_clamped - seg_lo) / max(seg_hi - seg_lo, 1e-6);
    ptr_x = 0.05 + (seg-1)*0.18 + frac*0.18;
    set(H.svZonePtr,'XData',ptr_x,'YData',0.217,'Color',abCol,'MarkerFaceColor',abCol);

    % State panel text
    set(H.svABDist,   'String',sprintf('%.2f m',ab_dist),'Color',abCol);
    set(H.svCommZone, 'String',zoneStr,'Color',abCol);

    %% Target markers ────────────────────────────────────────────────────────
    TGT_CLR  = [1.00 0.85 0.10];
    TGT_CLR2 = [1.00 0.50 0.05];
    dist3 = sqrt((S.x-T.x)^2+(S.y-T.y)^2+(S.z-T.z)^2);
    distH = sqrt((S.x-T.x)^2+(S.y-T.y)^2);   % horizontal only
    bearing_deg = atan2d(T.y-S.y, T.x-S.x);

    % Flash colour when captured
    if S.flash > 0
        tc = [0.4+0.6*(S.flash/12), 1.0, 0.4+0.3*(S.flash/12)];
    else
        tc = TGT_CLR;
    end

    % Side view: vertical dashed line at T.x, diamond at (T.x, T.z)
    set(H.hTgtLineS,'XData',[T.x T.x],'YData',[S.z-3 S.z+4]);
    set(H.hTgtMarkS,'XData',T.x,'YData',T.z,'Color',tc,'MarkerFaceColor',tc);

    % Top view: crosshair lines through (T.x,T.y), diamond
    set(H.hTgtLineT,'XData',[T.x T.x],'YData',[S.y-5 S.y+5]);
    set(H.hTgtMarkT,'XData',T.x,'YData',T.y,'Color',tc,'MarkerFaceColor',tc);

    % Front camera view: diamond at (T.y, T.z)
    set(H.hTgtLineF,'XData',[T.y T.y],'YData',[S.z-3 S.z+4]);
    set(H.hTgtMarkF,'XData',T.y,'YData',T.z,'Color',tc,'MarkerFaceColor',tc);

    % Capture ring around blimp (top view) when within 2x capture radius
    th_cap = linspace(0,2*pi,60);
    if distH < P.capture_r*3
        ring_r = P.capture_r;
        ring_alpha = max(0, 1 - distH/(P.capture_r*3));
        set(H.hCapRing,'XData',S.x+ring_r*cos(th_cap),'YData',S.y+ring_r*sin(th_cap),...
            'Color',TGT_CLR2);
    else
        set(H.hCapRing,'XData',NaN,'YData',NaN);
    end

    % 3D target sphere + equatorial ring
    r_cap = P.capture_r;
    [tsx2,tsy2,tsz2] = sphere(18);
    set(H.hTgt3,'XData',tsx2*r_cap+T.x,'YData',tsy2*r_cap+T.y,'ZData',tsz2*r_cap+T.z,...
        'FaceColor',tc,'EdgeColor','none','FaceAlpha',0.30+0.25*(S.flash>0));

    set(H.hTgtRing3,'XData',T.x+r_cap*cos(th_cap),'YData',T.y+r_cap*sin(th_cap),...
        'ZData',T.z*ones(size(th_cap)),'Color',TGT_CLR2);
    set(H.hTgtDrop3,'XData',[T.x T.x],'YData',[T.y T.y],'ZData',[-1 T.z]);
    set(H.hTgtFloor3,'XData',T.x,'YData',T.y,'ZData',-1,'Color',tc);

    % Score & distance HUD
    set(H.svScore,'String',num2str(S.score),'Color',tc);
    if dist3 < P.capture_r*2
        distCol = [0.3 1.0 0.4];
    else
        distCol = [0.17 1.0 0.56];
    end
    set(H.svDist,'String',sprintf('%.2f m',dist3),'Color',distCol);
    set(H.svBear,'String',sprintf('BRG %+.0f deg  dZ%+.2f',bearing_deg, T.z-S.z));

    %% State readouts ────────────────────────────────────────────
    vals=[S.x,S.y,S.z,S.psi*180/pi,S.u,S.w,S.r];
    wt  =[2.0,2.0,2.0,45,0.8,0.8,0.5];
    for i=1:7
        str=sprintf('%+.3f',vals(i));
        if abs(vals(i))>wt(i)
            set(H.sv(i),'String',str,'Color',[1 0.45 0.2]);
        else
            set(H.sv(i),'String',str,'Color',GRN);
        end
    end
    Fs=[inp.F1,inp.F2,inp.F3];
    for i=1:3
        pct=min(Fs(i)/P.F_max,1)*0.9+0.05;
        xd=get(H.mb(i),'XData'); xd(2)=pct; xd(3)=pct;
        set(H.mb(i),'XData',xd);
    end
end


function T = newTarget()
    % Spawn a new random target within reachable flight envelope
    T.x = (rand-0.5)*6;    % x in [-3, 3] m
    T.y = (rand-0.5)*6;    % y in [-3, 3] m
    T.z = rand*2.5 + 0.3;  % z in [0.3, 2.8] m (above ground)
end
