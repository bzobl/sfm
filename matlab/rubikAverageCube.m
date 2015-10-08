% run after 'sfm.m' for all variables to be set

good_view_idxs = [ ];
good_point_threshold = 10;
skip_if_rejected = 20;

views_kept = 0;

for view = 1 : size(views)
    
    view_1_idx = views(view, 1);
    view_2_idx = views(view, 2);
    
    eval(['pts = pts_' num2str(view_1_idx) '_' num2str(view_2_idx) ';']);
    eval(['errors = errors_' num2str(view_1_idx) '_' num2str(view_2_idx) ';']);
   
    % check which points are below threshold
    n_rejected = sum(errors > good_point_threshold);

    if (n_rejected > skip_if_rejected)
        fprintf('Skipping Views %d->%d\n', view_1_idx, view_2_idx);
        continue; 
    end
    
    fprintf('Keeping Views %d->%d\n', view_1_idx, view_2_idx);
    views_kept = views_kept + 1;
    pts_serialized(views_kept, :) = [ pts(1,:)./pts(4,:) pts(2,:)./pts(4,:) pts(3,:)./pts(4,:) ];
end

pts_serialized = mean(pts_serialized);
n_pts = size(pts, 2);
pts_avg = [pts_serialized(0*n_pts+1:1*n_pts); ...
           pts_serialized(1*n_pts+1:2*n_pts); ...
           pts_serialized(2*n_pts+1:3*n_pts); ...
           ones(1, n_pts); ];

%% Align 3d points
pt_o_idx = 6;       % point of origin
pt_x_idx = 36;      % target of x axis vector
pt_z_idx = 1;       % target of z axis vector

% offset to the the origin -> translation
t_o = - pts_avg(1:3,pt_o_idx) / pts_avg(4, pt_o_idx);

% scale so that red/bottom edge is exactly the length of a rubik's cube
rubik_side = 63.7;
subcube_side = rubik_side / 5;
avg_dist = rubikAverageDistance(pts_avg, zeros(1, n_pts));
s = subcube_side / avg_dist;

% scale for red/bottom axis to be exaxtly 1
% s = 1 / pdist([ points_3d(:,pt_x_idx) points_3d(:, pt_o_idx) ]', 'euclidean');

% get rotation matrix to align red/bottom axis with x axis
vx_1 = pts_avg(1:3,pt_x_idx) - pts_avg(1:3,pt_o_idx);
vx_1 = vx_1 / norm(vx_1);       % make it a unit vector
vx_2 = [ 1 0 0 ];
v = cross(vx_1, vx_2);
v_d = dot(vx_1, vx_2);
v_x = [ 0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
Rx = eye(3) + v_x + (v_x * v_x) * (1-v_d)/(norm(v)*norm(v));

% align red/backside axis at z axis by rotating about x axis     
z_side = pts_avg(1:3,pt_z_idx) - pts_avg(1:3,pt_o_idx);
Rz = rotationMatrix(z_side, [0 0 1]', [1 0 0]');

% apply to all points in pointcloud
for pt = 1 : size(points_3d, 2)
    R = s * Rz * Rx;
    pts_avg(:,pt) = [R, R * t_o; 0, 0, 0, 1] * pts_avg(:,pt);
end


%% Measure
[lx, ly, lz] = rubikMeasure(pts_avg, zeros(1, n_pts), 1);

rubik_side = 63.7;
fprintf('average & - & $%.2f{\\times}%.2f{\\times}%.2f$ & $%.2f$ & $%.2f$ & $%.2f$\\\\\n', ...
        lx, ly, lz, lx - rubik_side, ly - rubik_side, lz - rubik_side);

