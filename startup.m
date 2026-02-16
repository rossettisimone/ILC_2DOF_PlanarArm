% ILC_2DOF_PlanarArm - Add all MATLAB source and Simulink folders to path
% Run this script from the project root before using the code (e.g. in MATLAB: cd('path/to/ILC_2DOF_PlanarArm'); startup)
repo_root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(repo_root, 'matlab')));
addpath(fullfile(repo_root, 'simulink'));
