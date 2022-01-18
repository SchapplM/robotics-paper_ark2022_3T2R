% Reproduce the  final results from the dimensional synthesis. Generate
% robot images for all robots on the Pareto front. Use this to pick the
% most feasible robots for the paper

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Local run for creating all figures
% OptName = 'ARK_3T2R_20211231_smallplf3';
OptName = 'ARK_3T2R_20220102_collplfsphere4';

% Possible figures: See cds_vis_results_figures for figure codes.
settings_checkrepro = struct('eval_plots', {{''}}, ... % robvisu', 'jointtraj
  'update_template_functions', true); % sind lokal oft veraltet
cds_check_results_reproducability(OptName, [], settings_checkrepro); % 'P5RPRRR8V1G9P8A1'

%% Generate all figures on the cluster
if false % This section should be run manually
  clusterheaderfile=fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
    'dimsynth','dimsynth_cluster_header.m'); %#ok<UNRCH>
  jobdir = tmpDirFcn();
  targetfile = fullfile(jobdir, 'generate_robot_images.m');
  copyfile(clusterheaderfile, targetfile);
  fid = fopen(targetfile, 'a');
  fprintf(fid, 'parpool([1,8]);\n');
  fprintf(fid, 'parfevalOnAll(gcp(), @warning, 0, ''off'', ''MATLAB:prnRenderer:opengl'');\n');
  fprintf(fid, 'OptName = ''ARK_3T2R_20220102_collplfsphere4'';\n');
  fprintf(fid, 'settings_checkrepro = struct(''eval_plots'', {{''robvisu''}});\n');
  fprintf(fid, 'cds_check_results_reproducability(OptName, [], settings_checkrepro);\n');
  fclose(fid);
  cluster_repo_path = computingcluster_repo_path();
  addpath(cluster_repo_path);
  computation_name = sprintf('ARK_3T2RPKM_generate_robot_images_%s', ...
    datestr(now,'yyyymmdd_HHMMSS'));
  jobStart(struct( ...
    'name', computation_name, ...
    'ppn', 8, ... % request 8 nodes (see parpool initialization above)
    'matFileName', 'generate_robot_images.m', ...
    'locUploadFolder', jobdir, ...
    'time',6)); % hours
end
