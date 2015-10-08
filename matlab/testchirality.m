% TESTCHIRALITY - Tests whether the given camera matrix complies with the points
%
%
%
%

function [ success ] = testchirality(P, points_3d, verbose)
    if (nargin < 3)
        verbose = true;
    end
    n_points = size(points_3d, 2);
    
    points_in_view = 0;
    
    for idx = 1:n_points
        
        %reproject the point using given camera matrix and check wheter
        %weight is positive --> point in front of camera
        point_reprojected = P * points_3d(:,idx);
        if(point_reprojected(3) > 0)
            points_in_view = points_in_view + 1; 
        end
    end
    
    ratio = points_in_view / n_points;
    
    if (verbose)
        fprintf('%.2f%% of the points (%d / %d) are in front of the camera\n', ratio * 100, points_in_view, n_points);
    end
    
    if (ratio > 0.75)
        success = true;
    else
        success = false;
    end
end

