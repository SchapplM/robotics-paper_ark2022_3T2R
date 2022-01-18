% Textual description of the task for the dimensional synthesis
% Values printed here are used in the paper in the example section

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
OptName = 'ARK_3T2R_20220114_plfmorph';
resdirtotal = ark_dimsynth_data_dir();
d1 = load(fullfile(resdirtotal, OptName, [OptName,'_settings.mat']));
Set = d1.Set;
Traj_W = d1.Traj;
xm = mean(minmax2(Traj_W.X')'); %#ok<UDIM>
fprintf('Middle task position (in world frame): [%s]mm\n', disp_array(xm(1:3)*1e3,'%1.1f'));
fprintf('Middle task orientation (in world frame): [%s]deg\n', disp_array(xm(4:5)*180/pi,'%1.1f'));

deltax = minmax2(Traj_W.X')' - repmat(xm,2,1);
assert(all(abs(deltax(1,:)+deltax(2,:))<1e-10), 'assumed that motion is symmetric.');
fprintf('Task position increment (in world frame): [%s]mm\n', disp_array(deltax(1,1:3)*1e3,'%1.1f'));
fprintf('Task angle increment (in world frame): [%s]deg\n', disp_array(deltax(1,4:5)*180/pi,'%1.1f'));

RobNr = 7;
RobName = 'P5PRRRR6V1G10P8A1';
resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', RobNr, RobName));
tmp = load(resfile);
varlim = tmp.RobotOptRes.Structure.varlim;
varnames = tmp.RobotOptRes.Structure.varnames;

I_xyzbase = [find(strcmp(varnames, 'base x')); find(strcmp(varnames, 'base y')); ...
  find(strcmp(varnames, 'base z'))];
fprintf('Base position limits: [%s]mm ... [%s]mm\n', ...
  disp_array(1e3*varlim(I_xyzbase, 1)','%1.1f'), disp_array(1e3*varlim(I_xyzbase, 2)','%1.1f'));

I_baserad = find(strcmp(varnames, 'base radius'));
fprintf('Base diameter limits: %1.1fmm ... %1.1fmm\n', ...
  2*1e3*varlim(I_baserad, 1), 2*1e3*varlim(I_baserad, 2));

I_plfrad = find(strcmp(varnames, 'platform radius'));
fprintf('Platform diameter limits: %1.1fmm ... %1.1fmm\n', ...
  2*1e3*varlim(I_plfrad, 1), 2*1e3*varlim(I_plfrad, 2));


%% Get kinematic parameters of the robots
datadir = fullfile(fileparts(which('ark_dimsynth_data_dir.m')),'data');
tmp = load(fullfile(datadir, 'robot_groups.mat'));
RobotGroups = tmp.RobotGroups;
npkinminmax = NaN(1,2);
nparminmax = NaN(1,2);
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups{i,1};
  if RobotGroups{i,3} == 0, continue; end % no results
  fprintf('Read data for PKM group %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  erg = load(fullfile(datadir, sprintf('detail_result_group_%s.mat', GroupName)));
  Ipkin = contains(erg.Structure.varnames, 'pkin');
  fprintf('Group %d: %d parameters: {%s}\n', i, length(erg.Structure.varnames), ...
    disp_array(erg.Structure.varnames, '%s'));
  fprintf('Group %d: %d Kinematic Parameters: {%s}\n', i, sum(Ipkin), ...
    disp_array(erg.Structure.varnames(Ipkin), '%s'));
  npkinminmax = minmax2([npkinminmax, sum(Ipkin)]);
  nparminmax = minmax2([nparminmax, length(erg.Structure.varnames)]);
end
fprintf('%d to %d optimization parameters, %d to %d kinematic parameters\n', ...
  nparminmax(1), nparminmax(2), npkinminmax(1), npkinminmax(2));

%% Get computation details
fprintf('Computation time max. %1.1f hours. %d Gen., %d Ind.\n', ...
  Set.general.computing_cluster_max_time/3600, Set.optimization.MaxIter, ...
  Set.optimization.NumIndividuals);
tablepath = fullfile(resdirtotal, OptName, sprintf('%s_results_table.csv', OptName));
ResTab = readtable(tablepath, 'HeaderLines', 1);
ResTab_headers = readtable(tablepath, 'ReadVariableNames', true);
ResTab.Properties.VariableNames = ResTab_headers.Properties.VariableNames;
% ResTab.num_succ + ResTab.num_fail
% Note: Field for duration is not correct it the optimization is aborted
% due to time limit.
Dauer_minmax = minmax2(ResTab.comptime_sum(ResTab.Fval_Opt<1e3)');
fprintf('Duration of optimization: %1.1fh to %1.1fh.\n', Dauer_minmax(1)/3600, ...
  Dauer_minmax(2)/3600);
n_gen_calc = (ResTab.num_succ(ResTab.Fval_Opt<1e3) + ...
  ResTab.num_fail(ResTab.Fval_Opt<1e3))/Set.optimization.NumIndividuals;
fprintf('Number of generations performed until walltime limit: %d to %d (of %d).\n', ...
  floor(min(n_gen_calc))-1, floor(max(n_gen_calc))-1, Set.optimization.MaxIter);