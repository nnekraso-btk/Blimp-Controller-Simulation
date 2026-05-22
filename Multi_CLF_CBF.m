function Multi_CLF_CBF()
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
P.m_surge = 0.460 + 0.1091;
P.m_heave = 0.460 + 0.3120;
P.Iz      = 0.0873 + 0.0197;
P.Xu = 0.1900;  P.Zw = 0.3366;  P.Nr = 0.2450;
P.ly = 0.461;
P.F_max  = 1.07;
P.F_step = 0.40;
P.F_vert = 0.55;
P.a = 0.55;  P.b = 0.22;  P.c = 0.22;
 
% Comm zones
P.comm_min_red    = 0.80;
P.comm_min_yellow = 1.20;
P.comm_ideal_lo   = 1.20;
P.comm_ideal_hi   = 2.50;
P.comm_max_yellow = 3.00;
P.comm_max_red    = 4.00;
 
% Autonomous leader path  (figure-8 lemniscate)
P.path_A     = 2.5;    % amplitude [m]
P.path_omega = 0.22;   % angular speed [rad/s]  — one loop ~28 s
P.path_z     = 1.0;    % constant cruise altitude [m]
 
% Autonomous follower controller gains  (proportional only for clarity)
P.follow_dist = 1.60;  % desired separation [m] — sits in ideal zone centre
% Follower controller gains
% Kp  : N/m      — force per metre of position error
% Kd  : N/(m/s)  — force per (m/s) of gap closing/opening rate
% Kff : N/(m/s)  — force per (m/s) of leader-follower velocity mismatch
P.Kp      = 1.80;   % position error gain
P.Kd      = 0.60;   % derivative (reactive) gain
P.Kff     = 0.80;   % feedforward (predictive) gain
P.Kp_heave= 1.40;   % heave (altitude) proportional gain — independent axis
P.Kp_yaw  = 3.50;   % yaw gain for autoLeader only
P.Kd_yaw  = 0.40;   % yaw derivative for autoLeader only
% CLF-CBF QP safety filter
% CBF boundaries: use yellow-zone edges so QP pushes back before red
P.cbf_d_min   = P.comm_min_yellow;  % [m] inner barrier boundary (0.80 m)
P.cbf_d_max   = P.comm_max_yellow;  % [m] outer barrier boundary (3.20 m)
% ECBF decay rates: higher = stiffer boundary, lower = earlier pre-warning
P.cbf_alpha1  = 1.20;   % inner ECBF first  decay rate  [1/s]
P.cbf_alpha2  = 1.20;   % inner ECBF second decay rate  [1/s]
P.cbf_beta1   = 0.80;   % outer ECBF first  decay rate  [1/s]
P.cbf_beta2   = 0.80;   % outer ECBF second decay rate  [1/s]
% CLF exponential convergence rates
P.clf_gamma1  = 1.50;   % CLF first  decay rate  [1/s]
P.clf_gamma2  = 1.50;   % CLF second decay rate  [1/s]
% No slack (delta=0): CLF is a hard constraint — no obstacles so full convergence required
% If CLF causes QP infeasibility (motor saturation), falls back to CBF-only
P.cbf_enabled = true;   % set false to disable filter and compare behavior
 
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
xlim(axS,[-5 5]); ylim(axS,[-1 3]);
 
axT = ax2D([365 375 335 310],'X [m]','Y [m]','TOP  (X - Y)');
xlim(axT,[-5 5]); ylim(axT,[-5 5]);
 
axF = ax2D([18   45 335 300],'Y [m]','Z [m]','FRONT CAMERA  (Y - Z)');
xlim(axF,[-4 4]); ylim(axF,[-1 3]);
 
ax3 = axes('Parent',fig,'Units','pixels','Position',[365 45 500 300], ...
    'Color',[0.02 0.04 0.03],'XColor',DGR,'YColor',DGR,'ZColor',DGR, ...
    'GridColor',[0.07 0.20 0.11],'GridAlpha',0.8, ...
    'XGrid','on','YGrid','on','ZGrid','on','Box','on', ...
    'FontName','Courier New','FontSize',7,'Projection','perspective');
hold(ax3,'on'); view(ax3,38,22);
xlim(ax3,[-5 5]); ylim(ax3,[-5 5]); zlim(ax3,[-1 3]);
title(ax3,'3D PERSPECTIVE','Color',GRN,'FontName','Courier New','FontSize',9);
xlabel(ax3,'X','Color',DGR); ylabel(ax3,'Y','Color',DGR); zlabel(ax3,'Z','Color',DGR);
 
%% ── Right panel ───────────────────────────────────────────────────────────
axP = axes('Parent',fig,'Units','pixels','Position',[878 45 192 650], ...
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
text(axP,0.28,0.645,'LEADER (A)','Color',GRN,'FontName','Courier New', ...
    'FontSize',7,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');
text(axP,0.75,0.645,'FOLLOWER (B)','Color',CYN,'FontName','Courier New', ...
    'FontSize',7,'FontWeight','bold','HorizontalAlignment','center','Units','normalized');
 
sNm={'x','y','z','psi','u','w','r'};
sUn={'m','m','m','deg','m/s','m/s','r/s'};
svA=gobjects(7,1); svB=gobjects(7,1);
for i=1:7
    yp=0.59-(i-1)*0.075;
    text(axP,0.04,yp+0.03,sprintf('%s[%s]',upper(sNm{i}),sUn{i}), ...
        'Color',DGR,'FontName','Courier New','FontSize',6,'Units','normalized');
    svA(i)=text(axP,0.04,yp,'0.00','Color',GRN,'FontName','Courier New', ...
        'FontSize',9,'FontWeight','bold','Units','normalized');
    svB(i)=text(axP,0.54,yp,'0.00','Color',CYN,'FontName','Courier New', ...
        'FontSize',9,'FontWeight','bold','Units','normalized');
end
 
% Controller error readout (Mode 2)
text(axP,0.5,0.05,'CTRL ERROR','Color',DGR,'FontName','Courier New', ...
    'FontSize',7,'HorizontalAlignment','center','Units','normalized');
svCtrlErr = text(axP,0.5,0.015,'dx -- dy -- dz --','Color',[0.7 0.7 0.5], ...
    'FontName','Courier New','FontSize',7,'HorizontalAlignment','center','Units','normalized');
 
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
plot(axS,[-12 12],[-0.5 -0.5],'--','Color',[0.45 0.12 0.12],'LineWidth',0.8);
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
plot(axF,[-6 6],[0 0],'--','Color',[0.45 0.12 0.12],'LineWidth',0.8);
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
 
% Autonomous path preview (Mode 1 only)
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
 
% Comm zone rings (centered on A, top view + 3D)
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
H.svModeBanner=svModeBanner; H.svModeDesc=svModeDesc; H.svModeHint=svModeHint;
H.svABDist=svABDist; H.svCommZone=svCommZone; H.svZonePtr=svZonePtr;
H.svA=svA; H.svB=svB; H.svCtrlErr=svCtrlErr;
 
%% ═══════════════════════════════════════════════════════════
%% INITIAL STATE
%% ═══════════════════════════════════════════════════════════
SA = fmState(0,   0,   0, 0);   % Agent A at origin
SB = fmState(0, 2.0,   0, 0);   % Agent B offset 2 m in Y
 
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
 
        % ── Agent A ──────────────────────────────────────────
        if mode_ == 1
            % Mode 1: autonomous leader follows figure-8
            [inpA, xwpt, ywpt] = autoLeader(SA_, P_, dt);
        else
            % Mode 2: manual leader (arrow keys)
            inpA = manualInputs(SA_, P_);
            xwpt = NaN; ywpt = NaN;
        end
        SA_ = rk4step(SA_, inpA, P_, dt);
        % Store leader velocity so follower can estimate leader acceleration
        SA_.vAx_prev = SA_.u * cos(SA_.psi);
        SA_.vAy_prev = SA_.u * sin(SA_.psi);
        SA_.t     = SA_.t + dt;
        SA_.frame = SA_.frame + 1;
        if mod(SA_.frame,3)==0
            SA_.trail_x(end+1)=SA_.x; SA_.trail_y(end+1)=SA_.y; SA_.trail_z(end+1)=SA_.z;
            if length(SA_.trail_x)>400
                SA_.trail_x=SA_.trail_x(2:end); SA_.trail_y=SA_.trail_y(2:end); SA_.trail_z=SA_.trail_z(2:end);
            end
        end
 
        % ── Agent B ──────────────────────────────────────────
        if mode_ == 2
            % Mode 2: autonomous follower tracks A
            [inpB, des_pos, ctrl_err] = autoFollower(SB_, SA_, P_);
        else
            % Mode 1: manual follower (WASD)
            inpB = manualInputs(SB_, P_);
            inpB.cbf_h1=NaN; inpB.cbf_h2=NaN; inpB.cbf_active=false; inpB.clf_slack=0;
            des_pos = struct('x',NaN,'y',NaN,'z',NaN);
            ctrl_err= [0 0 0];
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
    S.u=0; S.w=0; S.r=0;
    S.keys=struct('up',0,'down',0,'left',0,'right',0);
    S.trail_x=[]; S.trail_y=[]; S.trail_z=[];
    S.frame=0; S.t=0;
    S.prev_epsi=0;   % yaw derivative term for autoLeader
    S.ex_prev=0;     % previous X position error — for Kd term
    S.ey_prev=0;     % previous Y position error — for Kd term
    S.vAx_prev=0;    % previous leader inertial X velocity — for leader accel estimate
    S.vAy_prev=0;    % previous leader inertial Y velocity — for leader accel estimate
end
 
%% ── Manual inputs (arrow keys or WASD, same physics) ────────────────────────
function inp = manualInputs(S, P)
    F1 = S.keys.right * P.F_step;
    F2 = S.keys.left  * P.F_step;
    Fz = (S.keys.up - S.keys.down) * P.F_vert;
    inp.Fx=F1+F2; inp.Fz=Fz; inp.Mz=(F1-F2)*P.ly;
    inp.F1=F1; inp.F2=F2; inp.F3=abs(Fz);
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
 
    % Where should B be? Directly behind the leader's tail.
    % We use the leader's heading angle to define "behind" in world coordinates.
    xd = L.x - P.follow_dist * cos(L.psi);
    yd = L.y - P.follow_dist * sin(L.psi);
    zd = L.z;
    des_pos.x=xd; des_pos.y=yd; des_pos.z=zd;
 
    % How far off is B from that spot right now?
    ex = xd - S.x;
    ey = yd - S.y;
    ez = zd - S.z;
 
    % Reactive term: how fast is the gap currently opening or closing?
    % Computed by differencing the position error between this tick and last.
    % If the gap is growing fast, this pushes harder before it gets worse.
    dex = (ex - S.ex_prev) / dt;
    dey = (ey - S.ey_prev) / dt;
 
    % Predictive term: if the leader is moving faster than B right now,
    % the gap will grow even if it hasn't started yet. This term fires
    % immediately on a velocity mismatch before any position error develops.
    % Both velocities are converted from body-frame speed + heading to
    % inertial X/Y components so they can be compared directly.
    vLx = L.u * cos(L.psi);
    vLy = L.u * sin(L.psi);
    vBx = S.u * cos(S.psi);
    vBy = S.u * sin(S.psi);
    dvx = vLx - vBx;
    dvy = vLy - vBy;
 
    % Add all three contributions in inertial frame.
    % Keeping them separate until here ensures units stay consistent
    % (each term independently produces Newtons before being summed).
    Fx_inertial = P.Kp*ex + P.Kd*dex + P.Kff*dvx;
    Fy_inertial = P.Kp*ey + P.Kd*dey + P.Kff*dvy;
 
    % Altitude is handled separately — no horizontal-vertical coupling
    % on this vehicle so a simple proportional controller is sufficient.
    F_heave = P.Kp_heave * ez;
 
    % Before sending forces to the motors, run them through the safety filter.
    % The filter checks whether the commanded force would violate either
    % communication boundary, and if so finds the closest safe alternative.
    % If the filter is off or unavailable, forces pass through unchanged.
    if P.cbf_enabled
        [Fx_inertial, Fy_inertial, cbf_info] = ...
            clf_cbf_filter(Fx_inertial, Fy_inertial, S, L, [xd;yd], P);
        inp.cbf_h1     = cbf_info.h1;
        inp.cbf_h2     = cbf_info.h2;
        inp.cbf_active = cbf_info.active;
        inp.clf_slack  = cbf_info.slack;
        inp.qp_status  = cbf_info.status;
    else
        inp.cbf_h1=NaN; inp.cbf_h2=NaN; inp.cbf_active=false;
        inp.clf_slack=0; inp.qp_status='DISABLED';
    end
 
    % The inertial force vector now needs to be split into what the motors
    % can actually produce. Surge (along the nose) is direct. The lateral
    % component — the part perpendicular to the nose — cannot be produced
    % directly, so it gets converted into a yaw moment instead. The blimp
    % turns toward the desired direction, then surges.
    % Negative surge is now allowed so B can brake rather than circle.
    F_surge   =  Fx_inertial*cos(S.psi) + Fy_inertial*sin(S.psi);
    F_lateral = -Fx_inertial*sin(S.psi) + Fy_inertial*cos(S.psi);
    Mz = F_lateral * P.ly;   % F × moment arm = torque
 
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
    function d = deriv(s, inp, P)
        d.du   = (inp.Fx - P.Xu*s.u) / P.m_surge;
        d.dw   = (inp.Fz - P.Zw*s.w) / P.m_heave;
        d.dr   = (inp.Mz - P.Nr*s.r) / P.Iz;
        d.dx   = s.u*cos(s.psi);
        d.dy   = s.u*sin(s.psi);
        d.dz   = s.w;
        d.dpsi = s.r;
    end
    k1=deriv(S,inp,P);
    s2=struct('x',S.x+dt/2*k1.dx,'y',S.y+dt/2*k1.dy,'z',S.z+dt/2*k1.dz, ...
        'psi',S.psi+dt/2*k1.dpsi,'u',S.u+dt/2*k1.du,'w',S.w+dt/2*k1.dw,'r',S.r+dt/2*k1.dr);
    k2=deriv(s2,inp,P);
    s3=struct('x',S.x+dt/2*k2.dx,'y',S.y+dt/2*k2.dy,'z',S.z+dt/2*k2.dz, ...
        'psi',S.psi+dt/2*k2.dpsi,'u',S.u+dt/2*k2.du,'w',S.w+dt/2*k2.dw,'r',S.r+dt/2*k2.dr);
    k3=deriv(s3,inp,P);
    s4=struct('x',S.x+dt*k3.dx,'y',S.y+dt*k3.dy,'z',S.z+dt*k3.dz, ...
        'psi',S.psi+dt*k3.dpsi,'u',S.u+dt*k3.du,'w',S.w+dt*k3.dw,'r',S.r+dt*k3.dr);
    k4=deriv(s4,inp,P);
    S.x   =S.x   +dt/6*(k1.dx  +2*k2.dx  +2*k3.dx  +k4.dx);
    S.y   =S.y   +dt/6*(k1.dy  +2*k2.dy  +2*k3.dy  +k4.dy);
    S.z   =S.z   +dt/6*(k1.dz  +2*k2.dz  +2*k3.dz  +k4.dz);
    S.psi =S.psi +dt/6*(k1.dpsi+2*k2.dpsi+2*k3.dpsi+k4.dpsi);
    S.u   =S.u   +dt/6*(k1.du  +2*k2.du  +2*k3.du  +k4.du);
    S.w   =S.w   +dt/6*(k1.dw  +2*k2.dw  +2*k3.dw  +k4.dw);
    S.r   =S.r   +dt/6*(k1.dr  +2*k2.dr  +2*k3.dr  +k4.dr);
    % Carry prev_epsi (not in deriv)
    if isfield(S,'prev_epsi'), S.prev_epsi=S.r; end
end
 
%% ── Clamp helper ─────────────────────────────────────────────────────────────
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
 
    % Scroll axes to midpoint of both agents
    mx=(SA.x+SB.x)/2; my=(SA.y+SB.y)/2; mz=(SA.z+SB.z)/2;
    xlim(H.axS,mx+[-5 5]);  ylim(H.axS,mz+[-1.5 2.5]);
    xlim(H.axT,mx+[-5 5]);  ylim(H.axT,my+[-5 5]);
    xlim(H.axF,my+[-4 4]);  ylim(H.axF,mz+[-1 2.5]);
    xlim(H.ax3,mx+[-5 5]);  ylim(H.ax3,my+[-5 5]);  zlim(H.ax3,mz+[-1.5 2.5]);
 
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
        set(H.svCtrlErr,'String',sprintf('ex%+.2f ey%+.2f ez%+.2f\nh1=%+.2f h2=%+.2f\n%s  %s',...
            ctrl_err(1),ctrl_err(2),ctrl_err(3),...
            inpB.cbf_h1,inpB.cbf_h2,cbf_str,qs));
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
    vA=[SA.x,SA.y,SA.z,SA.psi*180/pi,SA.u,SA.w,SA.r];
    vB=[SB.x,SB.y,SB.z,SB.psi*180/pi,SB.u,SB.w,SB.r];
    for i=1:7
        set(H.svA(i),'String',sprintf('%+.2f',vA(i)));
        set(H.svB(i),'String',sprintf('%+.2f',vB(i)));
    end
end
 
 
%% ── CLF-CBF safety filter ────────────────────────────────────────────────────
% Every tick, before being read by the motors, the follower's computed forces
% pass through the filter. If the forces will cause B to cross the boundary
% the goal is to minimize the required change to make it safe again.
%
% Two barrier functions define the limit zones> The inner prevents B from getting
% too close (collision risk), the outer prevents it from drifting too far (comm loss).
% A third constraint, the CLF, ensures B is always actively converging toward
% the formation position rather than just hovering near the boundary.
%
% No slack is used here since there are no static obstacles. If the CLF and
% CBF constraints genuinely conflict (e.g. during a sharp turn), the CLF is dropped 
% and CBF takes priority.
%
% Requires the MATLAB Optimization Toolbox to run. If unavailable, forces pass
% through unchanged and the HUD shows NO TOOLBOX.
 
function [Fx_s, Fy_s, info] = clf_cbf_filter(Fx_nom, Fy_nom, S, L, des_xy, P)
 
    dt = 0.025;
    m  = P.m_surge;
 
    % Work in acceleration space (divide by mass once here, multiply back at end).
    % This keeps the QP decision variables dimensionally clean.
    ax_nom = Fx_nom / m;
    ay_nom = Fy_nom / m;
 
    % Convert body-frame speeds to inertial X/Y components for both agents.
    vBx = S.u * cos(S.psi);
    vBy = S.u * sin(S.psi);
    vAx = L.u * cos(L.psi);
    vAy = L.u * sin(L.psi);
 
    % Leader acceleration is estimated by finite difference on stored velocity.
    % Needed because the barrier RHS terms include leader acceleration, and
    % without it the constraint would be wrong whenever the leader is accelerating.
    aAx = (vAx - L.vAx_prev) / dt;
    aAy = (vAy - L.vAy_prev) / dt;
 
    % Relative position and velocity; everything is B minus A so that
    % positive values mean B is ahead/faster, negative means behind/slower.
    dx   = S.x - L.x;
    dy   = S.y - L.y;
    d_sq = dx^2 + dy^2;
    dvx  = vBx - vAx;
    dvy  = vBy - vAy;
    dv_sq= dvx^2 + dvy^2;
    dp_dv= dx*dvx + dy*dvy;   % relative position dotted with relative velocity
 
    % h1 and h2 are the two barrier functions. Both are positive when safe, zero at the
    % boundary, and negative if violated. Using squared distance avoids a square
    % root and keeps the derivatives simple.
    % h1 check if blimp B is safely outside the danger-close inner circle
    % h2 checks if B is safely inside the comm-loss outer circle (sign inverts)
    h1     =  d_sq - P.cbf_d_min^2;
    h1_dot =  2 * dp_dv;    % positive = gap growing = moving away from A
    h2     =  P.cbf_d_max^2 - d_sq;
    h2_dot = -2 * dp_dv;    % positive = gap shrinking = moving toward A
 
    % The formation target isn't a fixed point, rather it moves with the leader.
    % When the leader turns, the point behind its tail sweeps in an arc, adding
    % a rotational component on top of the translational velocity.
    % Troubleshooting this fixed the CLF being invisible in testing.
    vdx = vAx + P.follow_dist * sin(L.psi) * L.r;
    vdy = vAy - P.follow_dist * cos(L.psi) * L.r;
 
    % CLF: V measures how far B is from where it should be.
    % The constraint forces V to decrease, so B always converges.
    epx  = S.x - des_xy(1);
    epy  = S.y - des_xy(2);
    V    = epx^2 + epy^2;
 
    % V_dot must use the velocity of B relative to the moving target, not
    % just B's velocity. A bug (now fixed) caused a stationary B to look like it's
    % converging when the target is moving away.
    V_dot = 2*epx*(vBx - vdx) + 2*epy*(vBy - vdy);
    ev_sq = (vBx-vdx)^2 + (vBy-vdy)^2;   % used in the second-order CLF term
 
    % Set up the QP. Cost = squared distance from nominal acceleration.
    % This allows minimal change to the force while staying safe.
    H_qp = 2 * eye(2);
    f_qp = [-2*ax_nom; -2*ay_nom];
 
    % Each constraint row says: this linear function of [ax, ay] must be
    % on the right side of a boundary. Everything on the right-hand side
    % is known from the current state — only [ax, ay] are unknown.
 
    % Inner barrier: don't let B accelerate toward A when already close.
    % The alpha terms add a pre-warning zone and the constraint tightens before
    % the boundary is actually reached, not only at it.
    rhs1 = 2*dv_sq ...
         - 2*(dx*aAx + dy*aAy) ...
         + (P.cbf_alpha1+P.cbf_alpha2)*h1_dot ...
         + P.cbf_alpha1*P.cbf_alpha2*h1;
 
    % Outer barrier: same structure but signs flip because the geometry
    % is inverted. Safe distance means being inside this circle.
    rhs2 = -2*dv_sq ...
         + 2*(dx*aAx + dy*aAy) ...
         + (P.cbf_beta1+P.cbf_beta2)*h2_dot ...
         + P.cbf_beta1*P.cbf_beta2*h2;
 
    % CLF: B must be converging toward the formation position.
    % The gamma terms control urgency (higher value = faster convergence,
    % but more aggressive force commands near the desired position).
    % Leader acceleration on the target point is approximated as zero here;
    % this introduces small error during sharp maneuvers but stays stable.
    rhs3 = -(2*ev_sq ...
         + (P.clf_gamma1+P.clf_gamma2)*V_dot ...
         + P.clf_gamma1*P.clf_gamma2*V);
 
    % Constraint matrix assembly. Each row is one constraint.
    % The columns correspond to [ax, ay], the two unknowns.
    a_max  = P.F_max / m;
    A_ineq = [-2*dx,  -2*dy;    % inner barrier (must push away from A)
               2*dx,   2*dy;    % outer barrier (must push toward A)
               2*epx,  2*epy];  % CLF (must converge toward formation spot)
    b_ineq = [rhs1; rhs2; rhs3];
    lb = [-a_max; -a_max];   % motor limits as acceleration bounds
    ub = [ a_max;  a_max];
 
    % Solve. Warm-start from nominal; usually only a small correction needed.
    z0   = [ax_nom; ay_nom];
    opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
    ax_s = ax_nom;  ay_s = ay_nom;   % default: pass through if QP fails
    cbf_active = false;
    clf_active = false;
    qp_status  = 'NOMINAL';
 
    try
        % First try: full CLF + CBF (all 3 constraints)
        [z_sol, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, ...
                                        [], [], lb, ub, z0, opts);
        if exitflag == 1
            ax_s = z_sol(1);
            ay_s = z_sol(2);
            deviation = norm([ax_s-ax_nom, ay_s-ay_nom]);
            cbf_active = deviation > 0.05;
            clf_active = cbf_active;
            if cbf_active, qp_status = 'QP ACTIVE'; end
        else
            % CLF may be causing infeasibility — try CBF-only (safety first)
            [z_sol2, ~, exitflag2] = quadprog(H_qp, f_qp, ...
                A_ineq(1:2,:), b_ineq(1:2), [], [], lb, ub, z0, opts);
            if exitflag2 == 1
                ax_s = z_sol2(1);
                ay_s = z_sol2(2);
                cbf_active = norm([ax_s-ax_nom, ay_s-ay_nom]) > 0.05;
                qp_status = 'CBF ONLY';
            else
                qp_status = 'INFEASIBLE';
            end
        end
    catch
        qp_status = 'NO TOOLBOX';
    end
 
    Fx_s = ax_s * m;
    Fy_s = ay_s * m;
 
    info.h1      = h1;
    info.h2      = h2;
    info.V       = V;
    info.V_dot   = V_dot;
    info.active  = cbf_active;
    info.slack   = 0;        % no slack in this formulation
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
    a=P.a; b=P.b; c=P.c; N=E0.n;
    gw=0.16; gh=0.08; gw3=0.14; gd3=0.22; gh3=0.09;
    t=linspace(0,2*pi,40);
    cp=cos(S.psi); sp=sin(S.psi);
    pv=atan2(S.w,max(0.05,abs(S.u)))*0.35;
    R=[cp -sp 0;sp cp 0;0 0 1];
 
    % Side view
    ex=S.x+a*cos(t)*cos(pv)-c*sin(t)*sin(pv);
    ez=S.z+a*cos(t)*sin(pv)+c*sin(t)*cos(pv);
    set(hEnvS,'XData',ex,'YData',ez);
    set(hGndS,'XData',[S.x-gw/2 S.x+gw/2 S.x+gw/2 S.x-gw/2], ...
        'YData',[S.z-c-gh S.z-c-gh S.z-c S.z-c]);
    set(hDirS,'XData',S.x,'YData',S.z,'UData',cos(pv)*0.48,'VData',sin(pv)*0.48);
    if length(S.trail_x)>1, set(hTrailS,'XData',S.trail_x,'YData',S.trail_z); end
 
    % Top view
    bx=a*cos(t); by=b*sin(t);
    set(hEnvT,'XData',S.x+bx*cp-by*sp,'YData',S.y+bx*sp+by*cp);
    set(hDirT,'XData',S.x,'YData',S.y,'UData',cp*0.48,'VData',sp*0.48);
    if length(S.trail_x)>1, set(hTrailT,'XData',S.trail_x,'YData',S.trail_y); end
    set(hLblT,'Position',[S.x S.y+0.4]);
 
    % Front camera
    aw=b*abs(cp)+a*abs(sp);
    set(hEnvF,'XData',S.y+min(aw,a)*cos(t),'YData',S.z+c*sin(t));
    if length(S.trail_y)>1, set(hTrailF,'XData',S.trail_y,'YData',S.trail_z); end
 
    % 3D surface
    Xr=zeros(N,N); Yr=Xr; Zr=Xr;
    for row=1:N
        for col=1:N
            v=R*[E0.x(row,col);E0.y(row,col);E0.z(row,col)];
            Xr(row,col)=v(1)+S.x; Yr(row,col)=v(2)+S.y; Zr(row,col)=v(3)+S.z;
        end
    end
    set(hSurf,'XData',Xr,'YData',Yr,'ZData',Zr);
 
    % Gondola box
    cb=[-gd3/2 -gw3/2 -gh3; gd3/2 -gw3/2 -gh3; gd3/2 gw3/2 -gh3; -gd3/2 gw3/2 -gh3;
        -gd3/2 -gw3/2 0;   gd3/2 -gw3/2 0;   gd3/2 gw3/2 0;   -gd3/2 gw3/2 0];
    cw=(R*cb')'; cw(:,1)=cw(:,1)+S.x; cw(:,2)=cw(:,2)+S.y; cw(:,3)=cw(:,3)+S.z-c;
    fg=[1 2 3 4;5 6 7 8;1 2 6 5;3 4 8 7;1 4 8 5;2 3 7 6];
    set(hGond3,'Vertices',cw,'Faces',fg);
 
    % Tail fins
    fL=[[-a*.85 0 0];[-a*1.08 b*.85 0];[-a*.65 b*.35 0]];
    fR=[[-a*.85 0 0];[-a*1.08 -b*.85 0];[-a*.65 -b*.35 0]];
    fT=[[-a*.85 0 0];[-a*1.08 0 c*.85];[-a*.65 0 c*.35]];
    for ii=1:3
        fL(ii,:)=(R*fL(ii,:)')'+[S.x S.y S.z];
        fR(ii,:)=(R*fR(ii,:)')'+[S.x S.y S.z];
        fT(ii,:)=(R*fT(ii,:)')'+[S.x S.y S.z];
    end
    set(hFin3L,'XData',fL(:,1),'YData',fL(:,2),'ZData',fL(:,3));
    set(hFin3R,'XData',fR(:,1),'YData',fR(:,2),'ZData',fR(:,3));
    set(hFin3T,'XData',fT(:,1),'YData',fT(:,2),'ZData',fT(:,3));
    set(hDir3,'XData',S.x,'YData',S.y,'ZData',S.z,'UData',cp*0.65,'VData',sp*0.65,'WData',0);
    if length(S.trail_x)>1
        set(hTrail3,'XData',S.trail_x,'YData',S.trail_y,'ZData',S.trail_z);
    end
end
