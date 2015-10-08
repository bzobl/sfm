% VISUALIZECAMERA Summary of this function goes here
%   Detailed explanation goes here
function [ output_args ] = visualizeCamera( fig, view_num, K, P, img_width, img_height)

    dX = 12.5;
    alpha_c = K(1,2);
    fc(1) = K(1,1); fc(2) = K(2,2);
    cc(1) = K(1,3); cc(2) = K(2,3);
    R = P(:, 1:3)
    t = P(:, 4)
    
    IP = 2*dX*[1 -alpha_c 0;0 1 0;0 0 1]*[1/fc(1) 0 0;0 1/fc(2) 0;0 0 1]*[1 0 -cc(1);0 1 -cc(2);0 0 1]*[0 img_width-1 img_width-1 0 0 ; 0 0 img_height-1 img_height-1 0;1 1 1 1 1];
    BASE = 2*(.9)*dX*([0 1 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 1]);
    IP = reshape([IP;BASE(:,1)*ones(1,5);IP],3,15);
    POS = [[6*dX;0;0] [0;6*dX;0] [-dX;0;5*dX] [-dX;-dX;-dX] [0;0;-dX]];

    BASEk = R' * (BASE - t * ones(1, 6));
    IPk = R' * (IP - t * ones(1, 15));
    POSk = R' * (POS - t * ones(1, 5));
    
    figure(fig);
    rotate3d on; grid on;
    xlabel('X_{world}'); ylabel('Y_{world}'); zlabel('Z_{world}');
    axis('equal'); axis vis3d; axis tight;
    view(50, 20);
    
    % plot origin
    plot3(3*dX*[1 0 0 0 0], 3*dX*[0 0 1 0 0], 3*dX*[0 0 0 0 1], 'r-', 'linewidth', 3);
    
    % draw camera
    f_len = -0.11;                                      % distance from camera center to imageplane
    ip_width = 1;                                   % width of imageplane
    ip_height = 1 * (img_height / img_width);       % height of imageplane
    cam_pts = [0 0 0; ...                           % center (= base) of camera
               -ip_width/2  ip_height/2 f_len; ...  % upper left corner
                ip_width/2  ip_height/2 f_len; ...  % upper right corner
               -ip_width/2 -ip_height/2 f_len; ...  % lower left corner
                ip_width/2 -ip_height/2 f_len; ...  % lower right corner
               ];
           
    %cam_pts = reshape([cam_pts;BASE(:,1)*ones(1,5);IP],3,15)
    cam_pts = R' * (cam_pts' - t * ones(1, 5));
    
    body_mesh = struct ('vertices', cam_pts', 'faces', [1 2 4; 1 4 5; 1 3 5; 1 2 3]);
    body = patch(body_mesh);
    set(body, 'facecolor', 'yellow', 'EdgeColor', 'black');
    imageplane_mesh = struct('vertices', cam_pts(:, 2:end)', 'faces', [1 2 3; 2 3 4]);
    imageplane = patch(imageplane_mesh);
    set(imageplane, 'facecolor', 'red', 'EdgeColor', 'none');    
    
    % show camera
%     p1 = struct('vertices', IPk','faces', [1 4 2; 2 4 7; 2 7 10; 2 10 1]);
%     IPk
%     visualize3DPoints(fig, IPk);
%     h1 = patch(p1);
%     set(h1, 'facecolor', [52 217 160]/255, 'EdgeColor', 'r');
%     
%     p2 = struct('vertices', IPk','faces', [1 10 7; 7 4 1]);
%     h2 = patch(p2);
%     set(h2, 'facecolor', [247 239 7]/255, 'EdgeColor', 'none');
%     
%     plot3(BASEk(1,:), BASEk(2,:), BASEk(3,:), 'b-', 'linewidth', 1);
%     plot3(IPk(1,:), IPk(2,:), IPk(3,:), 'r-', 'linewidth', 1);
%     text(POSk(1, 5), POSk(2, 5), POSk(3, 5), num2str(view_num), 'fontsize', 10, 'color', 'k', 'FontWeight', 'bold');
end

