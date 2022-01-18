% Führe die kombinierte Struktur- und Maßsynthese für alle 3T2R-PKM durch.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

usr_cluster = true;
%% Optimierung aller Roboter für alle Aufgabentypen starten

% Einstellungen der Optimierung
Set = cds_settings_defaults(struct('DoF', logical([1 1 1 1 1 0])));
Set.general.verbosity = 4;
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.task.maxangle = 10*pi/180;
Traj = cds_gen_traj(Set.task.DoF, 4, Set.task);
% Offset zu Winkeln dazugeben
Traj.X(:,4:5) = Traj.X(:,4:5) + 20*pi/180;
Traj.XE(:,4:5) = Traj.XE(:,4:5) + 20*pi/180;

% Debug: Nehme eine Aufgabe fast ohne Bewegungsspielraum
% Traj.XD = Traj.XD * 0.1;
% Traj.XDD = Traj.XDD * 0.1;
% Traj.X = repmat(Traj.X(1,:), size(Traj.X,1),1)+cumtrapz(Traj.t, Traj.XD);
% Traj.XE = Traj.X(Traj.IE,:);

% Verschiebe die Aufgabe
task_dim = (max(Traj.XE(:,1:3)) - min(Traj.XE(:,1:3)));
task_mid1 = min(Traj.XE(:,1:3)) + task_dim/2;
delta_x_task = -task_mid1; % Erst auf Ursprung ziehen
delta_x_task = delta_x_task + [0,0,1.5]; % höher ziehen (über Tisch)
Traj.XE(:,1:3) = Traj.XE(:,1:3) + repmat(delta_x_task, size(Traj.XE,1),1);
Traj.X(:,1:3) = Traj.X(:,1:3) + repmat(delta_x_task, size(Traj.X,1),1);
% task_mid = min(Traj.XE(:,1:3)) + task_dim/2;

% moderate Anzahl an Wiederholungen (reicht für gute Konvergenz)
Set.optimization.NumIndividuals = 100;
Set.optimization.MaxIter = 200;
% Beschränkung auf parallel
Set.structures.use_serial = false;
Set.structures.use_parallel = true;
Set.structures.max_index_active_revolute = 1; % Nur aktives Gestellgelenk bei Drehantrieb
Set.structures.max_index_active_prismatic = 3; % RP oder UP als Beginn der Beinkette erlauben
% Parallel und auf Cluster
Set.general.computing_cluster = usr_cluster; % Auf Cluster rechnen
Set.general.computing_cluster_cores = 16;
Set.general.cluster_maxrobotspernode = Set.general.computing_cluster_cores;
Set.general.computing_cluster_max_time = 10*3600;
% Debug: Auf bestimmten Cluster-Job warten (für bessere Startwerte)
% Set.general.cluster_dependjobs = 8191909;

% Debug-Bilder (werden auf Cluster nicht erzeugt)
% Set.general.plot_details_in_fitness = 4e9; % Selbstkollision
% Set.general.plot_details_in_fitness = 1e4*5e3; % Konfigurationssprung
% Set.general.plot_robot_in_fitness = 4e9;
% Set.general.plot_robot_in_fitness = 5e5*1e4;
% Set.general.plot_robot_in_fitness = 4.5e9;
if usr_cluster
  Set.general.plot_robot_in_fitness = 0;
  Set.general.plot_details_in_fitness = 0;
end
% Erlaubt in Desktop-Modus den Abschluss abgebrochener Optimierungen
Set.general.matfile_verbosity = 3;
% Bilder nicht speichern
Set.general.save_robot_details_plot_fitness_file_extensions = {};
% Für jeden Roboter eine Animation erstellen
Set.general.animation_styles = {'3D'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.general.eval_figures = {'pareto_all_phys'};
Set.general.noprogressfigure = true;

% Auch singuläre PKM zulassen
Set.optimization.condition_limit_sing = inf;
Set.optimization.condition_limit_sing_act = inf;
Set.structures.use_parallel_rankdef = 0;
% Debug: Bewusst PKM untersuchen, die einen Rangverlust haben
Set.structures.use_parallel_rankdef = 0;
Set.optimization.objective = {'chainlength', 'condition'}; % Nehme den leichtesten Roboter (am kürzesten)
Set.optimization.objective = {'chainlength', 'installspace', 'condition'};
if Set.structures.use_parallel_rankdef > 0
  % Bei Rangverlust ist die Untersuchung der Konditionszahl sinnlos.
  Set.optimization.objective = {'chainlength', 'installspace'};
  % Die Aktuierung wird auch ignoriert, es geht nur um die Mechanismen
  Set.structures.max_index_active_revolute = 6;
end
% Debug: Bei erstem funktionierendem Ergebnis aufhören
% Set.optimization.obj_limit = 1e3*ones(length(Set.optimization.objective),1);

% Set.optimization.constraint_obj(4) = 1000; % max. Wert für Konditionszahl
% Keine Selbstkollisionen (sieht sonst unplausibel im Bild aus)
Set.optimization.constraint_collisions = true;
% Geringfügige Änderung der Basisposition bzgl. x-y-Ebene zum Vermeiden von
% Singularitäten. In z-Richtung mehr Änderung zulassen, falls Beine kurz
% sind.
Set.optimization.basepos_limits = [[-0.6,0.6];[-0.6,0.6];[0,0.8]];
Set.optimization.base_size_limits = [0.7, 1.5];
Set.optimization.platform_size_limits = [0.1, 0.4];
Set.general.debug_calc = false;
Set.optimization.base_morphology = true;
Set.optimization.platform_morphology = true;
Set.optimization.rotate_base = true;
Set.optimization.ee_rotation = false;

n_instspcbodies = 2;
Set.task.installspace = struct( ...
  'type', uint8(zeros(n_instspcbodies,1)), ... % Nummern der Geometrie-Typen, siehe Implementierung.
  'params', NaN(n_instspcbodies,10), ...
  'links', {cell(n_instspcbodies,1)});
% Definiere den Bauraum: Zylinder um die Aufgabe
r_Zylinder = 2.0; % Radius
h_Zylinder = 3;
p_Zylinder = [[0,0,0], ... % Erster Punkt (unten)
  [0,0,h_Zylinder], ... % Zweiter Punkt (oben)
  r_Zylinder, NaN(1,3)];
Set.task.installspace.type(1) =  uint8(2); % Zylinder
Set.task.installspace.params(1,:) = p_Zylinder;
Set.task.installspace.links(1) =  {1:6};  % Alle bewegten Teile des Roboters müssen im Bauraum sein
% Zusätzliche Einschränkung für die Basis: flacherer Zylinder. Damit können
% die Führungsschienen von Schubgelenken nicht nach unten abstehen
r_Zylinder2 = 2.0; % Radius
h_Zylinder2 = 1;
p_Zylinder2 = [[0,0,0], ... % Erster Punkt (unten)
  [0,0,h_Zylinder2], ... % Zweiter Punkt (oben)
  r_Zylinder2, NaN(1,3)];
Set.task.installspace.type(2) =  uint8(2); % Zylinder
Set.task.installspace.params(2,:) = p_Zylinder2;
Set.task.installspace.links(2) = {0};  % Alle bewegten Teile des Roboters müssen im Bauraum sein

% Debug: Visualisierung der Aufgabe
% cds_show_task(Traj, Set)
% return

% Debug: Nur einen Roboter erzeugen
% Set.structures.whitelist = {'P5RPRRR8V1G9P8A1'}; % S5RPRRR2 'P5RPRRR8V1G9P8A1'
% Set.structures.whitelist = {'P5PRRRR6V1G10P8A1'}; % 5PRUR senkrechte PR
% Set.structures.whitelist = {'P5PRRRR4V1G10P8A1'};
% Set.structures.whitelist = {'P5RRRRR6V1G9P8A4'};

% Debug: Struktur einschränken
% Set.structures.joint_filter = 'PRRRR';
% Set.structures.num_tech_joints = 4; % PRUR
% Set.structures.parrob_basejointfilter = 10;

% Sieht auf Bildern übersichtlicher aus im Gegensatz zu Deckenmontage
Set.structures.mounting_parallel = 'floor';
if Set.general.computing_cluster && length(Set.structures.whitelist) == 1
  warning('Es soll nur ein Roboter auf dem Cluster berechnet werden. Sicher?');
  pause(5); % Bedenkzeit
end
Set.optimization.optname = sprintf('ARK_3T2R_20220104_longrun');

% Debug: Abgebrochenen Durchlauf abschließen
Set.general.only_finish_aborted = false;
% Debug: Falls Berechnung abgebrochen ist und Videos nachträglich erstellt
% werden sollen
Set.general.regenerate_summary_only = false; 

% Debug:
% serroblib_create_template_functions({'S5PRRRR4'}, false);
% parroblib_create_template_functions({'P5PRRRR4V1G1P8A1'}, false);
% parroblib_addtopath({'P5PRRRR4V1G1P8A1'});
% matlabfcn2mex({'P5PRRRR4V1_invkin'});
% matlabfcn2mex({'P5PRRRR4V1_invkin3'});
% matlabfcn2mex({'P5PRRRR4V1_invkin_traj'});
% parroblib_update_template_functions({'P5PRRRR4V1G1P8A1'});

cds_start(Set,Traj);
