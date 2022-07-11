% Show the coordinate frames of specific robots to transfer them in the
% kinematic sketch

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

repodir = fileparts(which('ark3T2R_dimsynth_data_dir'));
if isempty(repodir)
  error(['You have to create a file ark3T2R_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = ark3T2R_dimsynth_data_dir();
datadir = fullfile(repodir,'data');
tablepath = fullfile(datadir, 'results_all_reps_pareto.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);


%% Loop through specific robot models and create+save kinematic sketches
ShortNamesList = {'5RPUR', '5PRUR'};
LongNamesList = {'', 'P5PRRRR6V1G10P8A1'};
for i = 1:length(ShortNamesList)
  RobShortName = ShortNamesList{i};
  fprintf('Create Robot image with frames for %s\n', RobShortName);
  I = strcmp(ResTab.Beschreibung, RobShortName);
  if ~isempty(LongNamesList{i})
    I = I & strcmp(ResTab.Name, LongNamesList{i});
  end
  II = find(I, 1, 'first');
  RobName = ResTab.Name{II};
  LfdNr = ResTab.LfdNr(II);
  parroblib_addtopath({RobName});
  OptName = ResTab.OptName{II};
  setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
  d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
  Set = cds_settings_update(d1.Set);
  [R, Structure] = cds_dimsynth_robot(Set, d1.Traj, d1.Structures{LfdNr}, true);
  resfile = fullfile(resdirtotal, OptName, ...
    sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  Set.general.verbosity = 4;
  Set.general.plot_robot_in_fitness = 1e3;
  Structure_tmp = Structure;
  Structure_tmp.q0_traj = tmp.RobotOptRes.q0;
  [fval_i_test, ~, Q] = cds_fitness(R, Set, d1.Traj, ...
    Structure_tmp, tmp.RobotOptRes.p_val, tmp.RobotOptRes.desopt_pval);
  fhdl = change_current_figure(200); % See cds_fitness_debug_plot_robot
  saveas(fhdl, fullfile(repodir, 'paper', 'figures', ['robot_', RobShortName, '_frames.fig']));
end