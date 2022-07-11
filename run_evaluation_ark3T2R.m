% Run the evaluation of the simulation results for the paper
% This creates the robot plots for the paper.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

this_path = fileparts( mfilename('fullpath') );
addpath(this_path);
addpath(fullfile(fileparts(which('ark3T2R_dimsynth_data_dir.m')), 'dimsynth')); % move to first position on search path
% Versuche laden
run(fullfile(fileparts(which('ark3T2R_dimsynth_data_dir.m')), 'dimsynth', ...
  'eval_figures_pareto.m')); close all;
% Die Namen müssen nur einmalig bestimmt werden (sofern sich die Ergebnisse
% nicht ändern
run(fullfile(fileparts(which('ark3T2R_dimsynth_data_dir.m')), 'dimsynth', ...
  'robot_names.m')); close all;
% Pareto-Bild (gruppiert)
run(fullfile(fileparts(which('ark3T2R_dimsynth_data_dir.m')), 'dimsynth', ...
  'eval_figures_pareto_groups.m')); close all;
% Roboter für Bilder auswählen und Bilder zeichnen
run(fullfile(fileparts(which('ark3T2R_dimsynth_data_dir.m')), 'dimsynth', ...
  'select_eval_robot_examples.m')); close all;
run(fullfile(fileparts(which('ark3T2R_dimsynth_data_dir.m')), 'dimsynth', ...
  'robot_images.m')); close all;
run(fullfile(fileparts(which('ark3T2R_dimsynth_data_dir.m')), 'dimsynth', ...
  'task_description.m')); close all;
