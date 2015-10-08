% TRIANGULATE - Triangulates points from view_0 and view_1
%
%   Detailed explanation goes here
%
% Arguments:
%           K               - Calibration matrix of camera
%           kc              - Vector containing the distortion coefficients of the camera
%           image_dim       - 1x2 vector containing the width and height of the image
%           keypoints_0     - 2D homogeneous coordinates of features in view_0
%           keypoints_1     - 2D homogeneous coordinates of features in view_1
%           P_0             - The 3x4 camera matrix of view_0
%           P_1             - The 3x4 camera matrix of view_1
%           verbose         - flag whether or not output should be printed
%
% Returns:
%           points_3d       - Triangulated 3D points in homogeneous coordinates
%           mean_repr_err   - mean error of triangulation
%           reprojection_errors - 1xN vector containing the projection error for each point
%
function [ points_3d, mean_repr_err, reprojection_errors ] = triangulate(K, kc, image_dim, keypoints_0, keypoints_1, P_0, P_1, verbose)

    if (nargin < 8)
        verbose = true;
    end

    n_keypoints = size(keypoints_0, 2);

    % check whether keypoints are feasable
    if (size(keypoints_1, 2) ~= n_keypoints)
        error('During triangulation: Number of keypoints do not match');
    end

    K_inv = inv(K);
    reprojection_errors = zeros(1, n_keypoints);
    points_3d = zeros(4, n_keypoints);

    epsilon = 1e-9;
    iteration_counter = zeros(1, n_keypoints);

    % process each keypoint
    for idx = 1 : n_keypoints

        % normalized image coordinates
        um_0 = K_inv * keypoints_0(:,idx);
        um_1 = K_inv * keypoints_1(:,idx);

        w_0 = 1;
        w_1 = 1;

        % iterative linear LS triangulation
        for i = 1 : 10
            iteration_counter(idx) = i;

            X = LinearLSTriangulation(um_0, P_0, um_1, P_1, w_0, w_1);

            points_3d(:,idx) = X;

            % recalculate weights
            temp_w_0 = P_0(3,:) * X;
            temp_w_1 = P_1(3,:) * X;

            if ((abs(w_0 - temp_w_0) <= epsilon) && ...
                (abs(w_1 - temp_w_1) <= epsilon))
                break;
            end

            w_0 = temp_w_0;
            w_1 = temp_w_1;
        end

        % reproject 3D point and compute error
        kp_reprojected_h = K * P_1 * X;
        % convert to eucledean
        kp_reprojected = [ kp_reprojected_h(1) / kp_reprojected_h(3); kp_reprojected_h(2) / kp_reprojected_h(3) ];
        % apply distortion to reprojected point
        kp_reprojected = distort(kp_reprojected, image_dim, [K(1,1) K(2,2)], kc, [K(1,3) K(2,3)], K(1,2));

        repr_error = norm(kp_reprojected - keypoints_1(1:2,idx));
        reprojection_errors(idx) = repr_error;
    end

    mean_repr_err = mean(reprojection_errors);
    max_repr_err = max(reprojection_errors);
    min_repr_err = min(reprojection_errors);

    mean_iter = mean(iteration_counter);
    max_iter = max(iteration_counter);
    min_iter = min(iteration_counter);
    non_converging = sum(iteration_counter > 9);

    if verbose
        fprintf(['Triangulation done: %d keypoints processed.\n' ...
                 '                    %f mean iterations (%d max, %d min, %d did not converge)\n' ...
                 '                    %f mean reprojection error (%f max, %f min)\n'], ...
                n_keypoints, ...
                mean_iter, max_iter, min_iter, non_converging, ...
                mean_repr_err, max_repr_err, min_repr_err);
    end
end


function [ X ] = LinearLSTriangulation(um_0, P_0, um_1, P_1, w_0, w_1)
    % solving homogeneous linear system Ax = B
    % derived from um_0 = P_0 * X and um_1 = P_1 * X

    u_0 = um_0(1);
    v_0 = um_0(2);
    u_1 = um_1(1);
    v_1 = um_1(2);

    A = [ 
         1/w_0 .* [P_0(1,1) - u_0 * P_0(3,1)  P_0(1,2) - u_0 * P_0(3,2)   P_0(1,3) - u_0 * P_0(3,3);
                   P_0(2,1) - v_0 * P_0(3,1)  P_0(2,2) - v_0 * P_0(3,2)   P_0(2,3) - v_0 * P_0(3,3); ];
         1/w_1 .* [P_1(1,1) - u_1 * P_1(3,1)  P_1(1,2) - u_1 * P_0(3,2)   P_1(1,3) - u_1 * P_1(3,3);
                   P_1(2,1) - v_1 * P_1(3,1)  P_1(2,2) - v_1 * P_0(3,2)   P_1(2,3) - v_1 * P_1(3,3); ];
       ];

   B = [
        1/w_0 .* [u_0 * P_0(3, 4) - P_0(1,4);
                  v_0 * P_0(3, 4) - P_0(2,4); ];
        1/w_1 .* [u_1 * P_1(3, 4) - P_1(1,4);
                  v_1 * P_1(3, 4) - P_1(2,4); ];
       ];

   X = A\B;
   X(4) = 1.0;
end