% Show properties of the kinematics model for 3T2R PKM
% Equation numbers in the comments refer to the numbers in the paper
% 
% See also: ParRob_class_example_3T2R_sym.m in the robotics toolbox repo:
% examples_tests/ParRob/ in https://github.com/SchapplM/robotics-toolbox

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-01
% (C) Institut für mechatronische Systeme, Leibniz Universität Hannover

clear
clc
%% Initialization
if isempty(which('serroblib_path_init.m'))
  error('Serial robot library not in search path.');
end
if isempty(which('parroblib_path_init.m'))
  error('Parallel robot library not in search path.');
end
this_dir = fileparts(which('ParRob_3T2R_mdl_proof.m'));
addpath(fullfile(this_dir, '..'));
if isempty(which('ark3T2R_dimsynth_data_dir'))
  error(['You have to create a file ark3T2R_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = ark3T2R_dimsynth_data_dir();
datadir = fullfile(fileparts(which('ark3T2R_dimsynth_data_dir.m')),'data');
if ~exist(fullfile(datadir, 'robot_groups.mat'), 'file')
  error('Run eval_figures_pareto_groups.m first');
end
tmp = load(fullfile(datadir, 'robot_groups.mat'));
RobotGroups = tmp.RobotGroups;
% Select 5-RPUR for evaluation. For other robots, see variable RobotGroups
GroupName = 'P5RPRRR8V1G';

%% Initialize Robot and load data
detailfile = fullfile(datadir, sprintf('detail_result_group_%s.mat', GroupName));
if ~exist(detailfile, 'file')
  error('Run select_eval_robot_examples.m first');
end
erg = load(detailfile);
R = erg.R;
Q = erg.Q;
parroblib_addtopath({R.mdlname});
%% Show properties of the kinematics model
q0 = Q(1,:)';
X0 = erg.X(1,:)';

% Definitions for indices on the kinematics constraints
I_constr2 = 1:6*R.NLEG;
I_constr2(4:6:end) = []; % always remove the fourth component (dependent z angle), see equ. 7
I_constr3 = 1:6*R.NLEG;
I_constr3(4) = [];

%% Compute IK for Initial Pose
% Set excluded Z Euler angle to zero as this must not have influence on
% the model
X0(6) = 0;
% The given value for q0 should already be correct. Compute IK using Newton
% Raphson algorithm (text below equ. 6) to check again.
[q, phi] = R.invkin_ser(X0, q0);
assert(all(abs(q-q0) < 1e-8), ['The initial value q0 from dimensional ', ...
  'synthesis should already have been correct for x0']);
% Compute full kinematic constraints according to equ. 5 of the paper
[Phi3_red,Phi3_full] = R.constr3(q, X0);
assert(all(size(Phi3_red)==[29,1]), 'Output 1 of constr3 has to be 29x1');
assert(all(size(Phi3_full)==[30,1]), 'Output 2 of constr3 has to be 30x1');
% Set the last Euler angle to the actual value. This is necessary for
% some parts of the implementation
X0_corr = X0;
X0_corr(6) = X0_corr(6) + Phi3_full(4);
% Berechne die Zwangsbedingungen nochmal neu mit korrigiertem X(6)
[~,Phi3_full_corr] = R.constr3(q, X0_corr); % mit Fuehrungsbeinkette
assert(abs(Phi3_full_corr(4)) < 1e-10, ['residual corresponding to the ', ...
  'Z Euler angle has to be zero after correction']);
% Compute the reduced kinematic constraints according to equ. 7 of the
% paper.
[Phi2_red,Phi2_full] = R.constr2(q, X0);
assert(all(size(Phi2_red)==[25,1]), 'Output 1 of constr2 has to be 25x1 sein');
assert(all(size(Phi2_full)==[30,1]), 'Output 2 of constr2 has to be 30x1 sein');
assert(all(abs(Phi2_red) < 1e-10), 'Reduced residual has to be zero');
% Re-compute the equation for the reduced residual without neglecting the
% Z component. For this, the corrected Euler angle has to be used
[Phi2_red_corr,Phi2_full_corr] = R.constr2(q, X0_corr);
assert(all(abs(Phi2_red_corr) < 1e-10), 'Reduced residual has to be zero');
assert(all(abs(Phi2_full_corr) < 1e-10), ['Full residual from derivation of ', ...
  'reduced residual (leg-platform instead of leg-leg) has to be zero']);
%% Plot result
figure(10);clf;
hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
view(3);
s_plot = struct( 'ks_legs', [R.I1L_LEG; R.I2L_LEG], 'ks_platform', 1:6, 'straight', 0);
R.plot(q, X0, s_plot );
title(R.mdlname, 'interpreter', 'none');

%% Differential kinematics via the full kinematic constraints
% Compute matrix delta_dq from equ. 6
[G_q_red_3,G_q_full_3] = R.constr3grad_q(q, X0);
% Compute matrix delta_dx from equ. 6
[G_x_red_3,G_x_full_3] = R.constr3grad_x(q, X0);
assert(all(size(G_q_red_3)==[29,25]), 'Output 1 of constr3grad_q has to be 29x25');
assert(all(size(G_q_full_3)==[30,25]), 'Output 2 of constr3grad_q has to be 30x25');
assert(all(size(G_x_red_3)==[29,5]), 'Output 1 of constr3grad_x has to be 29x5');
assert(all(size(G_x_full_3)==[30,6]), 'Output 2 of constr3grad_x has to be 30x6');
G_q_3 = G_q_full_3(I_constr3,:);
G_x_3 = G_x_full_3(I_constr3,1:5);
% Compute full-joint inverse Jacobian matrix inv(Jtilde) from text below equ. 6
Jinv_3 = G_q_3 \ G_x_3;
% Compute active-joint inverse Jacobian matrix inv(J) from text below equ. 6
% Selection indices R.I_qa correspond to matrix P_a
J_qa_x_3 = Jinv_3(R.I_qa,:);
assert(all(size(J_qa_x_3)==[5 5]), 'inverse Jacobian Matrix has to be 5x5');
fprintf('Group %s. %s: Rank of the %dx%d Jacobian: %d/%d. condition number: %1.2e\n', ...
  GroupName, R.mdlname, size(J_qa_x_3,1), size(J_qa_x_3,2), rank(J_qa_x_3), sum(R.I_EE), cond(J_qa_x_3));

%% Differential kinematics via the reduced kinematic constraints
% Compute matrix delta_dq from equ. 6 with reduced residual psi instead of
% full residual delta
[G_q_red_2,G_q_full_2] = R.constr2grad_q(q, X0);
% Compute matrix delta_dx from equ. 6 (with reduced residual)
[G_x_red_2,G_x_full_2] = R.constr2grad_x(q, X0);
assert(all(size(G_q_red_2)==[25,25]), 'Output 1 of constr2grad_q has to be 25x25');
assert(all(size(G_q_full_2)==[30,25]), 'Output 2 of constr2grad_q has to be 30x25');
assert(all(size(G_x_red_2)==[25,5]), 'Output 1 of constr2grad_x has to be 25x5');
assert(all(size(G_x_full_2)==[30,6]), 'Output 2 of constr2grad_x has to be 30x6');
G_q_2 = G_q_full_2(I_constr2,:);
G_x_2 = G_x_full_2(I_constr2,1:5);
% Compute inverse Jacobian matrix inv(Jtilde) from text below equ. 6
Jinv_2 = G_q_2 \ G_x_2;
J_qa_x_2 = Jinv_2(R.I_qa,:);
test_Jinv_23 = Jinv_2 - Jinv_3;
if any(abs(test_Jinv_23(:))>1e-6)
  error('Inverse Jacobian does not match between reduced and full residual');
end
