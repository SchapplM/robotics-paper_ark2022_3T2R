% Skript zur Erzeugung diverser Bilder für den Konferenzvortrag

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-06
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
close all
outputdir = fullfile(fileparts(which('ark3T2R_dimsynth_data_dir')), 'presentation');
assert(isfolder(outputdir), sprintf('Verzeichnis %s existiert nicht', outputdir));
%% Pareto-Bild für Optimierung eines einzelnen Roboters
% Vorher erzeugen durch interaktives Pareto-Bild
uiopen(fullfile(ark3T2R_dimsynth_data_dir(), 'ARK_3T2R_20220611_useold_rep1', ...
  'Rob7_P5PRRRR6V1G10P8A1', 'Rob7_P5PRRRR6V1G10P8A1_Pareto2D_phys.fig'), 1);
fhdl = gcf();
fchdl = get(fhdl, 'children');
delete(fchdl([3 7])); % axes löschen, von nicht interessanten Kombinationen
sgtitle('');
xlabel('size (length of leg chains)');
ylabel('performance (cond(J)');
set(gca, 'xticklabel', []); set(gca, 'yticklabel', []);
set_size_plot_subplot(fhdl, ...
  8,4,gca,...
  0.06,0.01,0.01,0.12,0,0);
ylim([1e2, 1.5e6]);
exportgraphics(fhdl, fullfile(outputdir, 'pareto_example_single_robot.png'), 'Resolution','800');

%% Bild mit Arbeitsraumkollision
close all
% Vorher Bild manuell erzeugen durch Haltepunkt in cds_constr_collisions_ws
fignames = {'P5RRPRR4G9P8A1_workspacecoll', 'P5RPRRR8V1G9P8A1_installspace', ...
  'P5RPRRR8V1G9P8A1_selfcoll'};
figdir = fullfile(fileparts(which('robsynth_projektablage_path.m')), ...
  '07_Praesentationen', '20220627_Vortrag_ARK_3T2R-PKM', 'Bilder');
for i = 3%1:3
  uiopen(fullfile(figdir, [fignames{i}, '.fig']), 1);
  fhdl = gcf();
  set(fhdl, 'color', 'w');
  title('');sgtitle('');
  if i == 1
    view([10, 12]);
    set(gca,'XTICKLABEL',{});set(gca,'YTICKLABEL', {});set(gca,'ZTICKLABEL',{});
    set(gca,'xtick',[],'ytick',[],'ztick',[]);
    set(get(gca, 'XAxis'), 'visible', 'off');
    set(get(gca, 'YAxis'), 'visible', 'off');
    set(get(gca, 'ZAxis'), 'visible', 'off');
    xlabel('');ylabel('');zlabel('');
  end
  if i == 2
    view([-25, 11]);
  end
  set_size_plot_subplot(fhdl, ...
    12,12,gca,...
    0,0,0,0,0,0)
  % Benutze export_fig. Bei exportgraphics stimmt die Skalierung der Achsen
  % nicht
  export_fig(fhdl, fullfile(outputdir, [fignames{i},'.png']), '-r200');
  % exportgraphics(fhdl, fullfile(outputdir, [fignames{i},'.png']), 'Resolution','800');
end