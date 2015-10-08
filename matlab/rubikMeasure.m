%RUBIKMEASURE Summary of this function goes here
%   Detailed explanation goes here
function [ len_x, len_y, len_z ] = rubikMeasure( points_3d, repr_errors, error_limit )

    rubik_side = 63.7;
    subcube_side = rubik_side / 5;
    avg_dist = rubikAverageDistance(points_3d, repr_errors, error_limit, false);
    avg_dist = round(avg_dist * 10000)/10000;
    
    s = subcube_side / avg_dist;
    
    pts_mm = points_3d .* s;
    
    len_x = pdist([ pts_mm(:,1)  pts_mm(:, 31) ]', 'euclidean');
    len_y = pdist([ pts_mm(:,1)  pts_mm(:, 67) ]', 'euclidean');
    len_z = pdist([ pts_mm(:,1) pts_mm(:, 6) ]', 'euclidean');
end

