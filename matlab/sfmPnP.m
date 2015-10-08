% Reset
close all; clc; clear;

%% Configuration of views

% Files containing the data gathered through caltech toolbox
calibration_result_file = 'calib_f0/Calib_Results.mat';
keypoints_extraction_file = 'rubik-cam-move/calib_data.mat';


initial_views = [ 1 2 ];
n_views = 3;
verbose = true;

% Load calibration variables
eval(['load ' calibration_result_file ';']);
eval(['load ' keypoints_extraction_file ';']);

% construct Calibration matrix K
K = [fc(1) alpha_c cc(1); ...
     0     fc(2)   cc(2); ...
     0     0       1];

%% Initial Pose estimation

% Load 2D keypoints from both views
eval(['pts_1 = x_' num2str(initial_views(1)) ';']);
eval(['pts_2 = x_' num2str(initial_views(2)) ';']);
n_keypoints = size(pts_1, 2);

% keypoints are given in 2D homogeneous coordinates
keypoints_0 = [ pts_1; ones(1, n_keypoints) ];
keypoints_1 = [ pts_2; ones(1, n_keypoints) ];

% Calculate Pose of cameras and triangulate
[ iP_0, iP_1, R_1, t_1, points_3d, repr_errors ] = cammatrix(K, kc, [nx ny], keypoints_0, keypoints_1, verbose);

%% Remaining views using PnP
 
for view_idx = 1 : n_views

	fprintf('-- Processing view %d ---\n', view_idx);

    % Load 2D keypoints from view
    eval(['pts = x_' num2str(view_idx) ';']);
    keypoints = [ pts; ones(1, n_keypoints) ];

    [R, t] = efficient_pnp(points_3d', keypoints', K);
    eval(['P_' num2str(view_idx) ' = [R t]']);
end

for view_idx = 1 : n_views
	for other_idx = 1 : n_views
        
        if (view_idx == other_idx)
            continue;
        end
        
        fprintf('-- Triangulating view %d and %d ---\n', view_idx, other_idx);
    
        eval(['P = P_' num2str(view_idx) ';' ]);
        eval(['kp = [ x_' num2str(view_idx) '; ones(1, n_keypoints) ];']);

        eval(['P_other = P_' num2str(other_idx) ';' ]);
        eval(['kp_other = [ x_' num2str(other_idx) '; ones(1, n_keypoints) ];']);

        [ pts_pnp, mean_repr_err, repr_err_pnp ] = triangulate(K, kc, [nx ny], kp, kp_other, P, P_other, verbose);

        visualizeRubik(1, pts_pnp, repr_err_pnp);
	end
end