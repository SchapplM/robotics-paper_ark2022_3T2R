% Video zum Zusammenbau einer 3T2R-PKM in der inversen Kinematik mit
% Hervorhebung der Koordinatensysteme.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

if isempty(which('parroblib_path_init.m'))
  warning('Repo mit parallelen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

usr_create_animation = true;
usr_debug_plot_ik = true;
usr_ikmode = 'parallel';

%% Roboter-Klasse vorbereiten
% Ergebnisse laden
datadir = fullfile(fileparts(which('ark3T2R_dimsynth_data_dir')), 'data');
respath = fullfile(fileparts(which('ark3T2R_dimsynth_data_dir')), 'presentation');
GroupName = 'P5RRRRR5V1G';
Name = '5-RRUR';
erg = load(fullfile(datadir, sprintf('detail_result_group_%s.mat', GroupName)));

%% Roboter initialisieren
R = erg.R;
% Dicke der Segmente reduzieren
for kk = 1:R.NLEG
  R.Leg(kk).DesPar.seg_par(:,1) = 1e-3;
  R.Leg(kk).DesPar.seg_par(:,2) = 1e-2;
end
parroblib_addtopath({R.mdlname});

X = erg.X(1,:)';

% Start-Gelenkwinkel wählen. TODO: Mit dieser Konfiguration aktuell noch
% falsche Konfiguration der ersten Beinkette als Ergebnis
q0_leg = zeros(R.Leg(1).NJ,1);
q0_leg(1) = pi/2;
q0_leg(2) = pi/3;
q0_leg(2) = pi/3;
q0 = repmat(q0_leg, R.NLEG, 1);
% Zufällige Anfangswerte
q0 = erg.Q(1,:)';
rng(0);
q0 = q0 + (-0.5+rand(length(q0),1));
q0_leg(1:R.Leg(1).NJ:R.NJ) = pi/2; % nach außen drehen

qlim_pkm = cat(1, R.Leg.qlim);

figure(2); clf; hold on; grid on; % Bild der Entwurfsparameter
xlabel('x in m');ylabel('y in m');zlabel('z in m'); view(3);
  s_plot = struct( 'ks_legs', [R.I1L_LEG; R.I1L_LEG+1; R.I2L_LEG], ...
    'ks_platform', [1:R.NLEG,R.NLEG+2], 'straight', 1, 'mode', 4);
R.plot( q0, X, s_plot );
title(sprintf('%s in Startkonfiguration', Name));

%% IK zum Startpunkt lösen
t1 = tic();
s = struct('retry_limit', 0, 'normalize', false, 'scale_lim', 0.7);
s.maxrelstep = 0.01; % sehr feinschrittige Bewegungen (für flüssige Animation)
s.n_max = 5e3;
if strcmp(usr_ikmode, 'serial')
  [q, Phi, ~, Stats] = R.invkin_ser(X, q0, s);
elseif strcmp(usr_ikmode, 'parallel')
  s.maxrelstep = 0.002;
  [q, Phi, ~, Stats] = R.invkin3(X, q0, s);
else
  error('Fall nicht vorgesehen')
end
fprintf('%s: IK berechnet. Dauer %1.1fs\n', R.mdlname, toc(t1));

% Berechne die Jacobi-Matrix bezogen auf 3T3R
X_neu = R.fkineEE_traj(q')'; % damit PKM-Jacobi mit Methode 4 stimmt (Orientierung neu bei 3T2R)

% Extrahiere Gelenkverlauf aus IK-Statistik
Q_t_Anfahrt = Stats.Q;
Q_t_norm = (Stats.Q-repmat(qlim_pkm(:,1)',size(Stats.Q,1), 1)) ./ ...
            repmat(qlim_pkm(:,2)'-qlim_pkm(:,1)',size(Stats.Q,1), 1);
% Verschiebe den Start der Bewegung der einzelnen Beinketten beim Modus
% Seriell-IK, damit verdeutlicht wird, dass die Bewegung nacheinander
% passiert
if length(Stats.iter)>1
  % Fülle mit NaN auf, falls die Anzahl der Schritte nicht ausreicht
  Q_t_Anfahrt = [Q_t_Anfahrt;NaN(sum(Stats.iter)-size(Q_t_Anfahrt,1),size(Q_t_Anfahrt,2))]; %#ok<AGROW>
  for i = 1:R.NLEG
    I_Legi = R.I1J_LEG(i):R.I2J_LEG(i);
    % Setze Gelenkwinkel auf den letzten Wert, wenn ein Wert NaN wird
    I_firstnan = find(any(isnan(Q_t_Anfahrt(:,I_Legi)),2),1,'first');
    if isempty(I_firstnan) || I_firstnan == 1, continue; end
    Q_t_Anfahrt(I_firstnan:end,I_Legi) = repmat(Q_t_Anfahrt(I_firstnan-1,I_Legi), ...
      size(Q_t_Anfahrt,1)-I_firstnan+1, 1);
  end
  % Setze erste Werte (während die vorherigen Beinketten sich bewegen)
  % auf den Anfangswert
  for i = 2:R.NLEG
    I_Legi = R.I1J_LEG(i):R.I2J_LEG(i);
    i_start = sum(Stats.iter(1:i-1))+1;
    Q_t_Anfahrt(i_start:i_start+Stats.iter(i)-1, I_Legi) = Q_t_Anfahrt(1:Stats.iter(i),I_Legi);
    Q_t_Anfahrt(1:i_start-1,I_Legi) = repmat(Q_t_Anfahrt(1,I_Legi),i_start-1,1);
  end
  % Kürze Daten auf tatsächliche Bewegung
  Q_t_Anfahrt = Q_t_Anfahrt(1:sum(Stats.iter),:);
else
  Q_t_Anfahrt = Q_t_Anfahrt(1:Stats.iter,:);
end
assert(all(abs(Q_t_Anfahrt(end,:)-q')<1e-3), ...
  'IK-Ergebnis passt nicht zu Anfahrt-Trajektorie');

% Permanent nur die Zielpose zeichnen (da IK-Methode 3T3R)
X_t_Anfahrt = repmat(X', size(Q_t_Anfahrt,1), 1);

% Verlauf der IK-Konvergenz prüfen
if usr_debug_plot_ik
  figure(10);clf;
  subplot(2,3,1);
  plot(Q_t_norm(1:max(Stats.iter),:));
  ylabel('Q norm'); grid on;
  subplot(2,3,2);
  plot(Stats.PHI(1:max(Stats.iter),:));
  ylabel('PHI'); grid on;
  subplot(2,3,3);
  plot(X_t_Anfahrt(1:max(Stats.iter),1:3));
  ylabel('x (transl)'); grid on;
  subplot(2,3,4);
  plot(X_t_Anfahrt(1:max(Stats.iter),4:6));
  ylabel('x (rot)'); grid on;
  subplot(2,3,5); hold on
  plot(Stats.condJ(1:max(Stats.iter)));
  if isfield(Stats, 'h')
    plot(Stats.h(1:max(Stats.iter),5));
    legend({'Phi_q', 'J_{PKM}'});
  end
  ylabel('cond(J)'); grid on;
  subplot(2,3,6);
  if isfield(Stats, 'h')
    plot(Stats.h(1:max(Stats.iter),:));
    ylabel('h'); grid on;
  end
  linkxaxes
  sgtitle(sprintf('IK-Diagnose'));
  figure(3); clf; hold on; grid on; % Bild der Entwurfsparameter
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m'); view(3);
  s_plot = struct( 'ks_legs', [R.I1L_LEG; R.I2L_LEG], ...
    'straight', 1, 'mode', 4, 'ks_platform', [1:R.NLEG,R.NLEG+2]);
  R.plot( q, X_neu, s_plot );
  title(sprintf('%s nach IK', Name));
end
if any(Stats.iter == s.n_max)
  warning(['Es wurden %d Iterationen gebraucht. Starkes Indiz für ', ...
    'Oszillationen am Ende oder schlechte Konvergenz.'], max(Stats.iter));
end
assert(all(abs(Phi)<1e-8), 'IK nicht erfolgreich');

%% Animation zeichnen
close all % damit nicht ins falsche Bild gezeichnet wird
if usr_create_animation
  % Zeit-Stützstellen für die Animation vorbereiten
  t_Anfahrt = (0:size(Q_t_Anfahrt,1)-1)'/size(Q_t_Anfahrt,1);
  t_plot = t_Anfahrt;
  Q_plot = Q_t_Anfahrt;
  X_plot = X_t_Anfahrt;
  
  maxduration_animation = 30; % Dauer des mp4-Videos in Sekunden (langsam, damit man folgen kann)
  t_Vid = (0:1/30*(t_plot(end)/maxduration_animation):t_plot(end))';
  I_anim = knnsearch( t_plot , t_Vid );
  I_anim = [I_anim; repmat(length(t_plot),15,1)]; % 15 Standbilder (0.5s) mit letztem Wert
  fprintf('Erstelle Animation mit %d/%d Zeit-Stützstellen\n', length(I_anim), length(t_plot));
  % Animation zeichnen
  anim_filename = fullfile(respath, sprintf('%s_IK_Assembly_%s',Name, usr_ikmode));
  s_anim = struct( 'mp4_name', [anim_filename,'.mp4']);%, 'resolution', 200); % 200dpi gibt ungefähr 1080p Breite nach Zuschnitt.
  % Keine Koordinatensysteme plotten. Macht das Bild zu unübersichtlich.
%   s_plot = struct( 'ks_legs', [R.I1L_LEG; R.I2L_LEG], ...
%     'ks_platform', [1:R.NLEG,R.NLEG+2], 'straight', 1, 'mode', 4);
  s_plot = struct( 'ks_legs', [], 'ks_platform', [], 'straight', 1, 'mode', 4);
  figure(1);clf;hold all;
  set(1, 'name', sprintf('Anim'), ...
    'color','w', 'NumberTitle', 'off', 'units','normalized',...
    'outerposition',[0 0 1 1]); % Vollbild, damit GIF größer wird
  view(3); axis auto; hold on;
  % Keine Achsbeschriftungen
  grid off;
  view(3);
  set(gca,'XTICKLABEL',{}, 'YTICKLABEL', {}, 'ZTICKLABEL',{});
  set(gca,'xtick',[],'ytick',[],'ztick',[])
  set(get(gca, 'XAxis'), 'visible', 'off');
  set(get(gca, 'YAxis'), 'visible', 'off');
  set(get(gca, 'ZAxis'), 'visible', 'off');
  set(gca, 'Box', 'off');
  R.anim( Q_plot(I_anim,:), X_plot(I_anim,:), s_anim, s_plot);
end
