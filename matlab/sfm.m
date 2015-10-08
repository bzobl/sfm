%% SfM - Structure from Motion
%
% Reconstructing 3D structure from subsequent taken images (single camera)

% Reset
close all; clc; clear;

%% Configuration of views

% Files containing the data gathered through caltech toolbox
calibration_result_file = 'calib_f0/Calib_Results.mat';
%keypoints_extraction_file = 'calib_f0/calib_data.mat'
%keypoints_extraction_file = 'pattern_big/calib_data.mat';
%keypoints_extraction_file = 'box/calib_data.mat';
%keypoints_extraction_file = 'rubik/calib_data.mat';
keypoints_extraction_file = 'rubik-cam-move/calib_data.mat';
%keypoints_extraction_file = 'cpp-keypoints.mat';

%views = [ [1 2]; [1 3]; [1 4]; [2 1]; [2 3]; [2 4]; [3 1]; [3 2]; [3 4]; [4 1]; [4 2]; [4 3]];
%views = [ [1 2]; [1 3]; [1 4]; [2 1]; [2 4]; [3 1]; [3 4]; [4 1]; [4 2]; [4 3]];
%views = [ [1 4]; [4 1];];
views = [ [1 3];];
align = false;
visualize = true;
verbose = true;
measure = true;
good_point_threshold = 25; %10000;
measure_reproj_error = 25;
skip_if_rejected = 20; %200;

% Load calibration variables
eval(['load ' calibration_result_file ';']);
eval(['load ' keypoints_extraction_file ';']);

% construct Calibration matrix K
K = [fc(1) alpha_c cc(1); ...
     0     fc(2)   cc(2); ...
     0     0       1];

for view_idx = 1 : size(views)

    view_1_idx = views(view_idx, 1);
    view_2_idx = views(view_idx, 2);

    if ~measure
        fprintf('-- Processing views %d and %d ---\n', view_1_idx, view_2_idx);
    else
        fprintf('\\hline\n');
    end

    % Load 2D keypoints from both views
    eval(['pts_1 = x_' num2str(view_1_idx) ';']);
    eval(['pts_2 = x_' num2str(view_2_idx) ';']);
    n_keypoints = size(pts_1, 2);

    % keypoints are given in 2D homogeneous coordinates
    keypoints_0 = [ pts_1; ones(1, n_keypoints) ];
    keypoints_1 = [ pts_2; ones(1, n_keypoints) ];

    % Calculate Pose of cameras and triangulate
    % Pose between view_0 and view_1
    % view_0 is assumed to be the origin with no rotation nor translation
    [ P_0, P_1, R_1, t_1, points_3d, repr_errors ] = cammatrix(K, kc, [nx ny], keypoints_0, keypoints_1, verbose);

    % check which points are below threshold
    n_rejected = sum(repr_errors > good_point_threshold);

    eval(['pts_' num2str(view_1_idx) '_' num2str(view_2_idx) ' = points_3d;']);
    eval(['errors_' num2str(view_1_idx) '_' num2str(view_2_idx) ' = repr_errors;']);

    %fprintf('With a maximal reprojection error of %f, %d points were rejected\n', good_point_threshold, n_rejected);

    if (skip_if_rejected >= 0) && (n_rejected > skip_if_rejected)
        fprintf('Too many rejected points %d/%d allowed. Skipping\n', n_rejected, skip_if_rejected);
        continue; 
    end

    if align
        %% Align 3d points
        pt_o_idx = 6;       % point of origin
        pt_x_idx = 36;      % target of x axis vector
        pt_z_idx = 1;       % target of z axis vector

        % offset to the the origin -> translation
        t_o = - points_3d(1:3,pt_o_idx) / points_3d(4, pt_o_idx);

       	% scale so that red/bottom edge is exactly the length of a rubik's cube
        rubik_side = 63.7;
        subcube_side = rubik_side / 5;
        avg_dist = rubikAverageDistance(points_3d, repr_errors, measure_reproj_error, verbose);
        s = subcube_side / avg_dist;

        % scale for red/bottom axis to be exaxtly 1
        % s = 1 / pdist([ points_3d(:,pt_x_idx) points_3d(:, pt_o_idx) ]', 'euclidean');

        % get rotation matrix to align red/bottom axis with x axis
        vx_1 = points_3d(1:3,pt_x_idx) - points_3d(1:3,pt_o_idx);
        vx_1 = vx_1 / norm(vx_1);       % make it a unit vector
        vx_2 = [ 1 0 0 ];
        v = cross(vx_1, vx_2);
        v_d = dot(vx_1, vx_2);
        v_x = [ 0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        Rx = eye(3) + v_x + (v_x * v_x) * (1-v_d)/(norm(v)*norm(v));

        % align red/backside axis at z axis by rotating about x axis     
        z_side = points_3d(1:3,pt_z_idx) - points_3d(1:3,pt_o_idx);
        Rz = rotationMatrix(z_side, [0 0 1]', [1 0 0]');

        % apply to all points in pointcloud
        for pt = 1 : size(points_3d, 2)
            R = s * Rz * Rx;
            points_3d(:,pt) = [R, R * t_o; 0, 0, 0, 1] * points_3d(:,pt);
        end
    end

    if visualize
        if sum(isnan(points_3d)) > 0
            fprintf('views %d -> %d contain NaN values\n', view_1_idx, view_2_idx);
        else
            visualizeRubik(1, points_3d, repr_errors);
        end
    end

    if measure
        rubik_side = 63.7;
        [len_x, len_y, len_z] = rubikMeasure(points_3d, repr_errors, measure_reproj_error);
        fprintf('%d $\\rightarrow$ %d & $%.4f$ & $%.2f{\\times}%.2f{\\times}%.2f$ & $%.2f$ & $%.2f$ & $%.2f$\\\\\n', ...
                view_1_idx, view_2_idx, mean(repr_errors), ...
                len_x, len_y, len_z, len_x - rubik_side, len_y - rubik_side, len_z - rubik_side);
    end
end