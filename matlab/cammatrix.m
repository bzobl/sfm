% CAMMATRIX - Computes camera matrices for view_0 and view_1
%   
% Function computes camera matrices for view_0 and view_1: P_0 and P_1
% respectively. view_0 is assumed to be without rotation nor translation,
% therefore P_0 will allways be [1 0 0 0; 0 1 0 0; 0 0 1 0]
%
% Arguments:
%           K               - The 3x3 Calibration Matrix
%           kc              - The distortion coefficients
%           image_dim       - 1x2 vector containing the width and height of the image
%           keypoints_0     - homogeneous coordinates of features in view_0
%           keypoints_1     - homogeneous coordinates of features in view_1
%           verbose         - flag whether output should be printed
%
% Returns:
%           P_0             - camera matrix of view_0. will always be in origin. Does not container K_0
%           P_1             - camera matrix of view_1. Does not container K_1
%           R_1             - rotation of view_1
%           t_1             - translation of view_1
%           points_3d       - triangulated 3D points in homogeneous coordinates
%           repr_error      - reprojection error for each point
%           rejected_points - 1xN vector containing 0 for valid point and 1 for rejected point
%

function [ P_0, P_1, R_1, t_1, points_3d, repr_err ] = cammatrix(K, kc, image_dim, keypoints_0, keypoints_1, verbose)

    if (nargin < 6)
        verbose = true;
    end
    n_keypoints = size(keypoints_0, 2);
    points_3d = zeros(4, n_keypoints);
    repr_err = zeros(1, n_keypoints);

    essential_matrix_feasable_threshold = 1e-07;
    repr_error_threshold = 50;

    % get Fundamental matrix of the two views
    [F, e1, e2] = fundmatrix(keypoints_0, keypoints_1);

    % calculate Essential matrix from Fundamental matrix
    E = K' * F * K;

    % check wether E is feasable
    if (abs(det(E)) > essential_matrix_feasable_threshold)
        error('The essential matrix is not feasable');
    end

    % initialize matrices
    P_0 = [ 1 0 0 0; 0 1 0 0; 0 0 1 0 ];
    P_1 = [ 1 0 0 0; 0 1 0 0; 0 0 1 0 ];
    
    % decompose Essential matrix in R and t
    % using SVD (for another solution see B. Horn's 1990 paper
    %'Recovering Baseline and Orientation from Essential Matrix')
    
    [ svd_U, svd_S, svd_V ] = svd(E);

    % check whether singular values are feasable
    s_ratio = svd_S(1, 1) / svd_S(2, 2);
    if (abs(s_ratio) > 1)
        %flip ratio
        s_ratio = 1 / s_ratio;
    end
    if (abs(s_ratio) < 0.7)
        error('While decomposing E: Singular values are not feasable');
    end

    R90 = [ 0 -1 0; 1 0 0; 0 0 1 ];

    % check whether U and V' are rotation matrices
    det_svd_U = det(svd_U);
    det_svd_Vt = det(svd_V');
    if (xor(abs(det_svd_U  + 1) < 1e-9, abs(det_svd_Vt + 1) < 1e-9))
        if verbose
            fprintf('While decomposing E: Either U or Vt is not a rotation matrix, flipping sign of U\n');
        end
        svd_U = -svd_U;
    end

    maybe_R1 = svd_U * R90 * svd_V';
    maybe_R2 = svd_U * R90' * svd_V';
    maybe_t1 = svd_U(:,end);  % last column of U
    maybe_t2 = -svd_U(:,end); % last column of U

    % assert that R really is a rotation matrix
    if (((abs(det(maybe_R1)) + 1) < 1e-9) || ...
        ((abs(det(maybe_R2)) + 1) < 1e-9))
        error('While decomposing E: R1 or R2 did not yield a valid rotation matrix\n');
    end
    
    % check all 4 combinations for P_1
    for comb = 1 : 4
        if verbose
            fprintf('While searching Cameramatrix: testing combination %d\n', comb);
        end
        
        switch comb
            case 1
                R_1 = maybe_R1;
                t_1 = maybe_t1;
            case 2
                R_1 = maybe_R1;
                t_1 = maybe_t2;
            case 3
                R_1 = maybe_R2;
                t_1 = maybe_t1;
            case 4
                R_1 = maybe_R2;
                t_1 = maybe_t2;
            otherwise
                error('severe error while trying P_1 combinations');
        end
        
        P_1 = [ R_1 t_1];
        if (verbose)
            disp(P_1);
        end
        
        [ points_3d_0, repr_err_0, errors_0 ] = triangulate(K, kc, image_dim, keypoints_0, keypoints_1, P_0, P_1, verbose);
        [ points_3d_1, repr_err_1, errors_1 ] = triangulate(K, kc, image_dim, keypoints_1, keypoints_0, P_1, P_0, verbose);
       
        if ((repr_err_0 > repr_error_threshold) || ...
            (repr_err_1 > repr_error_threshold) || ...
            (testchirality(P_1, points_3d_0, verbose) == false) || ...
            (testchirality(P_0, points_3d_1, verbose) == false))
            P_1 = 0;
            continue;
        end

        % a working combination was found
        break;
    end
    
    if (P_1 == 0)
        error('While searching Cameramatrix: None of the 4 combinations yielded a satisfying result');
    end
    
    if verbose
        fprintf('DONE\n');
    end
    
    for i = 1 : n_keypoints
        if (errors_0(i) < errors_1(i))
            points_3d(:,i) = points_3d_0(:,i);
            repr_err(i) = errors_0(i);
        else
            points_3d(:,i) = points_3d_1(:,i);
            repr_err(i) = errors_1(i);
        end
    end

end

