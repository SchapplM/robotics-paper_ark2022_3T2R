% Wandle die Namen der Roboter in ein lesbares Format um
% Berechne dazu die Fitness-Funktion einmalig um Gelenkwinkel für die
% Referenzpunkte zu bestimmen. Dadurch wird die Parallelität der Gelenke
% geprüft.
% 
% Vorher ausführen:
% * eval_figures_pareto.m
% 
% Erzeugt Datei:
% * robot_names_latex.csv
% 
% Quelle:
% [KongGos2007] Kong, X., Gosselin, C.M.: Type synthesis of parallel
% mechanisms. Springer Berlin Heidelberg (2007)
% 
% This script is based on the same file robot_names.m from 
% https://github.com/SchapplM/robsynth-paper_mhi2021
% (MHI paper "Combined Structural and Dimensional Synthesis of a Parallel
% Robot for Cryogenic Handling Tasks", Schappler et al. 2022)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

usr_writemode = 'edit'; % Bearbeite bestehende Liste nachträglich (nicht immer alles neu ausführen)
%% Definitionen
outputdir = fileparts(which('robot_names.m'));
datadir = fullfile(outputdir,'..','data');
if isempty(which('ark3T2R_dimsynth_data_dir'))
  error(['You have to create a file ark3T2R_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = ark3T2R_dimsynth_data_dir();
serroblibpath=fileparts(which('serroblib_path_init.m'));
%% Öffnen der Ergebnis-Tabelle
% (Wird in results_stack_tables.m erstellt)
tablepath = fullfile(datadir, 'results_all_reps_pareto.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);

%% Generiere die Zeichenfolge für die Gelenkkette
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
Robots = unique(ResTab.Name);
ResTab_NameTrans = cell2table(cell(0,7), 'VariableNames', {'PKM_Name', ...
  'Gnum', 'Pnum', 'Chain_Name', 'ChainStructure', 'Chain_Structure_Act', 'Chain_ShortName'});
if strcmp(usr_writemode, 'edit')
  ResTab_NameTrans = readtable(namestablepath, 'Delimiter', ';');
end

for i = 1:length(Robots) % find(strcmp(Robots, 'P4RPRRR12V1G1P1A1'))
  RobName = Robots{i};
  fprintf('Bestimme Bezeichnung für Rob %d (%s)\n', i, RobName);
  parroblib_addtopath({RobName});
  II_Robi = find(strcmp(ResTab.Name, RobName));
  j = II_Robi(1); % Lade das erste (geht nur um den Roboter selbst)

  %% Lade Ergebnis und Roboter
  OptName = ResTab.OptName{j};
  LfdNr = ResTab.LfdNr(j);
  setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
  d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
  Set = cds_settings_update(d1.Set);
  % Ergebnisse wurden ohne die Beschränkung auf symmetrische Schubzylinder
  % generiert. Daher Option hier deaktivieren.
  if ~isfield(d1.Set.optimization, 'joint_limits_symmetric_prismatic')
    Set.optimization.joint_limits_symmetric_prismatic = false;
  end
  % Debug: Falls es Probleme mit der Reproduzierbarkeit vorheriger Ergebnisse gibt.
%   Set.optimization.prismatic_cylinder_allow_overlength = true;
%   % Ergebnisse wurden ohne neues Plattform-Kollisionsobjekt erzeugt. Deaktivieren
%   Set.optimization.collshape_platform = {'ring'};
  % Kollisionsprüfung für Bestimmung der Parallelität nicht notwendig
  % (Kollisionen sollten eigentlich gleich bleiben bei Optimierung und
  % hier)
%   Set.optimization.constraint_collisions = false;

  resfile = fullfile(resdirtotal, OptName, ...
    sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  if any(tmp.RobotOptRes.fval > 1e3)
    warning('PKM hat bereits in Optimierung nicht funktioniert. Hätte aussortiert werden müssen');
    continue
  end
  resfile2 = fullfile(resdirtotal, OptName, ...
    sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
  if exist(resfile2, 'file')
    d2 = load(resfile2);
    RobotOptDetails = d2.RobotOptDetails;
  else
    RobotOptDetails = [];
  end
  q0 = tmp.RobotOptRes.q0;
  PName = tmp.RobotOptRes.Structure.Name;
  [~,LEG_Names, Actuation] = parroblib_load_robot(PName);
  parroblib_update_template_functions({PName});
  % Debug:
%   serroblib_create_template_functions(LEG_Names(1),false);
%   parroblib_create_template_functions({PName},false);
%   matlabfcn2mex({[PName(1:end-6),'_invkin']}); % einige Dateien werden hiermit doppelt kompiliert
%   Chain_Name = tmp.RobotOptRes.R.Leg(1).mdlname_var;
  Chain_Name = LEG_Names{1};
  NLegJ = str2double(Chain_Name(2));
%   tmp.RobotOptRes.R.Leg(1).NJ;
  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', NLegJ), sprintf('S%d_list.mat',NLegJ));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof');
  ilc = strcmp(l.Names_Ndof, Chain_Name);
  % Technische Gelenke bestimmen
  SName_TechJoint = fliplr(regexprep(num2str(l.AdditionalInfo(ilc,7)), ...
    {'1','2','3','4','5'}, {'R','P','C','U','S'}));
  %% Roboter-Klasse initialisieren
  [R, Structure] = cds_dimsynth_robot(Set, d1.Traj, d1.Structures{LfdNr}, true);
  % Anpassung für Programm-Aktualisierung seit Generierung der Ergebnisse
  p_val_corr = cds_parameters_update(tmp.RobotOptRes.Structure, ...
    Structure, tmp.RobotOptRes.p_val);
  % Korrigiere den axoffset-Parameter für neue Implementierung; ab 22.04.22
  if tmp.RobotOptRes.timestamps_start_end(1) < datenum(2022, 4, 22)
    I_pmao = strcmp(Structure.varnames, 'platform_morph_axoffset');
    I_pfr = Structure.vartypes == 7;
    p_val_corr(I_pmao) = p_val_corr(I_pmao) * p_val_corr(I_pfr);
  end
  % Parameter des Ergebnisses eintragen (für fkine-Berechnung unten)
  cds_update_robot_parameters(R, Set, Structure, p_val_corr);
  % Gelenkwinkel des Startwerts für IK eintragen
  for kk = 1:R.NLEG
    R.Leg(kk).qref = q0(R.I1J_LEG(kk):R.I2J_LEG(kk));
  end
  % Fitness-Funktion nachrechnen um Gelenk-Trajektorie zu bestimmen. Ändere
  % die Einstellungen, so dass keine Dynamik berechnet wird (geht schneller).
  clear cds_save_particle_details cds_fitness
  Structure_tmp = Structure;
  Structure_tmp.calc_cut = false;
  Structure_tmp.calc_dyn_act = false;
  Structure_tmp.calc_spring_act = false;
  Structure_tmp.calc_spring_reg = false;
  Structure_tmp.calc_dyn_reg = false;
  % Erzwinge Prüfung dieses Anfangswerts für Trajektorie (falls IK anderes
  % Ergebnis hat). Diese Option sollte nicht notwendig sein. Wird zur
  % Sicherheit trotzdem gemacht.
  Structure_tmp.q0_traj = q0;
  Set.optimization.objective = {'condition'};
  Set.optimization.constraint_obj(:) = 0;
  
  % Prüfe Kinematik-Parameter des Roboters aus den Detail-Ergebnis-Daten
  % (zum Debuggen, falls die Parameter nicht passen sollten)
  resfile2 = fullfile(resdirtotal, OptName, ...
    sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
  if exist(resfile2, 'file')
    tmp2 = load(resfile2);
    test_pkin = tmp2.RobotOptDetails.R.Leg(1).pkin - R.Leg(1).pkin;
    assert(all(abs(test_pkin)<1e-10), 'Kinematikparameter sind bei erneutem Laden anders als bei erster Durchführung')
  else
    fprintf('Datei %s existiert nicht (kein Fehler). Test gegen Detail-Daten aus erster Durchführung nicht möglich\n', resfile2)
  end

  % Debug: Bei Verletzung von Zielfunktionen Bilder zeichnen
  % Set.general.plot_robot_in_fitness = -1e3;
  % Set.general.plot_details_in_fitness = -1e3;
  % Keine Eingabe von Ergebnissen von Entwufsoptimierung.
  % Schubgelenk-Offsets hier neu berechnen (falls Konfiguration umklappt)
  Set.optimization.pos_ik_tryhard_num = 50; % bessere Reproduzierbarkeit in IK
  [fval_i_test, ~, Q] = cds_fitness(R, Set, d1.Traj, ...
    Structure_tmp, p_val_corr, tmp.RobotOptRes.desopt_pval);
  if any(abs(Q(1,:)'-q0) > 1e-3)
    % Debug: Zeichne ursprüngliche Konfiguration
%     Set.general.plot_robot_in_fitness = -1e3;
%     cds_fitness_debug_plot_robot(R, q0, [], d1.Traj, Set, Structure_tmp, p_val_corr, inf, '');
  end
  if any(fval_i_test > 1e3)
    % Eigentlich darf dieser Fall nicht vorkommen. Ist aber aus numerischen
    % Gründen leider doch manchmal möglich.
    warning('Die Nebenbedingungen wurden bei erneuter Prüfung verletzt');
    if any(fval_i_test > 1e11) || ... % siehe cds_constraints.
        any(fval_i_test < 1e9) && any(fval_i_test > 1e4) % siehe cds_constraints_traj.
      % Versuche nochmal neu, die Fitness-Funktion zu berechnen
      error(['Keine gültige Gelenkwinkel berechnet. Parallelität der ', ...
        'Gelenke und damit Name nicht bestimmbar.'])
    end
  end
  %% Parallelität der Gelenke anzeigen (anhand der Trajektorie).
  % Direkte Kinematik für alle Zeitschritte berechnen
  Zges = NaN(size(Q,1), 3*NLegJ); % Alle z-Achsen speichern zum späteren Vergleich
  pgroups_all = zeros(size(Q,1), NLegJ);
  sigma_leg = R.Leg(1).MDH.sigma;
  for k = 1:size(Q,1)
    Tc = R.Leg(1).fkine(Q(k,1:NLegJ)');
    for kk = 1:NLegJ
      Zges(k,(kk-1)*3+1:kk*3) = Tc(1:3,3,1+kk);
    end
    % Werte die Parallelität der Achsen aus
    for jj = 1:NLegJ
%       if sigma_leg(jj) == 1
        % Schubgelenk. Gruppe trotzdem zählen. Sonst würde die Gruppe nach
        % dem Schubgelenk wieder bei 1 anfangen
%         continue
      if jj == 1 % Erstes Gelenk ist per Definition immer Gruppe 1
        pgroups_all(k, jj) = 1;
        continue
      end
      for kk = 1:jj-1
        if sigma_leg(kk) == 1
          % Parallelität zum Schubgelenk wird betrachtet
          % Steht so nicht in [KongGos2007] drin. Wird hier neu eingeführt.
%           continue
        end
        % Prüfe welches die erste z-Achse ist, die identisch mit der
        % aktuellen ist
        z_jj = Zges(k,(jj-1)*3+1:jj*3);
        z_kk = Zges(k,(kk-1)*3+1:kk*3);
        if all(abs(z_jj-z_kk) < 1e-6) || all(abs(z_jj+z_kk) < 1e-6) % parallel oder antiparallel ist gleichwertig
          pgroups_all(k, jj) = pgroups_all(k, kk); % Gelenk jj ist parallel zu Gelenk kk
          break
        else % Debug
          % deltaphi = acos(dot(z_jj,z_kk));
          % fprintf('Beingelenk %d vs %d: %1.1f deg Verdreht\n', jj, kk, 180/pi*deltaphi);
        end
      end
      if pgroups_all(k, jj) == 0
        pgroups_all(k, jj) = max(pgroups_all(k, 1:jj-1)) + 1; % Neue Gruppe
      end
    end
    if k > 1 && any(pgroups_all(k-1,:) ~= pgroups_all(k,:))
      fprintf('Parallelität ändert sich in Zeitschritt %d (kein Fehler)\n', k);
      Set.general.plot_robot_in_fitness = -1e3;
      cds_fitness_debug_plot_robot(R, Q(k,:)', [], d1.Traj, Set, Structure_tmp, p_val_corr, inf, '');
    end
  end
  if any(any(diff(pgroups_all)))
    warning('Die Parallelität ändert sich im Zeitverlauf. Sollte nicht sein.');
    % Kann passieren, wenn Drehgelenke parallel zu einem Schubgelenk werden
    % Nehme den allgemeineren Fall mit mehr unterschiedlichen Gruppen
    I_maxdiv = max(pgroups_all, [], 2);
    pgroups = pgroups_all(find(I_maxdiv,1,'first'),:);
  else
    % Keine Änderung der Parallelität. Nehme die Gruppen vom ersten Fall.
    pgroups = pgroups_all(1,:);
  end

  %% Zeichenkette generieren. Siehe Kong/Gosselin 2007, S.10
  % Variablen mit Latex-Code für Roboter-Namen
  Chain_StructName = '';
  Chain_StructNameAct = '';
  
  groupidx = 0;
  for j = 1:NLegJ
    if ~any(pgroups(1:j-1) == pgroups(j)) && sum(pgroups == pgroups(j)) ~= 1
      % Diese Gruppe hat bei diesem Gelenk ihr erstes Vorkommnis
      groupidx = groupidx + 1; % hochzählen
    end
    if sum(pgroups == pgroups(j)) == 1
      % diese Gelenkausrichtung gibt es nur einmal. Es muss kein
      % Gruppensymbol darüber gelegt werden
      newsymbol = '{';
    elseif groupidx == 1
      newsymbol = '{\`';
    elseif groupidx == 2
      newsymbol = '{\''';
    elseif groupidx == 3
      newsymbol = '{\=';
    else
      error('mehr als drei Achsrichtungen nicht vorgesehen');
    end
    % Füge "P"/"R" hinzu
    newsymbol = [newsymbol, Chain_Name(2+j), '}']; %#ok<AGROW>
    Chain_StructName = [Chain_StructName, newsymbol]; %#ok<AGROW>
    if any(Actuation{1} == j)
      Chain_StructNameAct = [Chain_StructNameAct, '\underline']; %#ok<AGROW>
    end
    Chain_StructNameAct = [Chain_StructNameAct, newsymbol]; %#ok<AGROW>
  end
  
  % In Tabelle speichern
  Gnum = d1.Structures{LfdNr}.Coupling(1);
  Pnum = d1.Structures{LfdNr}.Coupling(2);
  I_found = strcmp(ResTab_NameTrans.RobName, RobName);
  Row_i = {RobName, Gnum, Pnum, Chain_Name, Chain_StructName, Chain_StructNameAct, SName_TechJoint};
  if any(I_found) % eintragen
    ResTab_NameTrans(I_found,:) = Row_i;
  else % anhängen
    ResTab_NameTrans = [ResTab_NameTrans; Row_i]; %#ok<AGROW>
  end
end

%% Speichere das wieder ab
writetable(ResTab_NameTrans, namestablepath, 'Delimiter', ';');
fprintf('Tabelle %s geschrieben\n', namestablepath);
