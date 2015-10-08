%VISUALIZERUBIK Summary of this function goes here
%   Detailed explanation goes here
function visualizeRubik(fig, points_3d, points_rejected)

    color_rejected = [255 102 0]./255;
    
    % red side
    visualize3DPoints(fig, points_3d(:,1:30), points_rejected(1:30), 'red', color_rejected);
    hold on;
    plot_edge(points_3d, 6, 1, 'red');
    plot_edge(points_3d, 1, 31, 'red');
    plot_edge(points_3d, 36, 6, 'red');
    
    % yellow side
    visualize3DPoints(fig, points_3d(:,31:66), points_rejected(31:66), 'yellow', color_rejected);
    hold on;
    plot_edge(points_3d, 31, 61, 'yellow');
    plot_edge(points_3d, 61, 66, 'yellow');
    plot_edge(points_3d, 66, 36, 'yellow');
    plot_edge(points_3d, 36, 31, 'yellow');
    
    % blue side
    visualize3DPoints(fig, points_3d(:,67:end), points_rejected(67:end), 'blue', color_rejected);
    hold on;
    plot_edge(points_3d, 67, 1, 'blue');
    plot_edge(points_3d, 67, 61, 'blue');
    
    oversize = 5 * pdist(points_3d(:,1:2)', 'euclidean');
    xlim([min(points_3d(1,:)) - oversize, max(points_3d(1,:)) + oversize]);
    ylim([min(points_3d(2,:)) - oversize, max(points_3d(2,:)) + oversize]);
    zlim([min(points_3d(3,:)) - oversize, max(points_3d(3,:)) + oversize]);
end

function plot_edge(points_3d, from, to, color)
    plot3([points_3d(1,from) points_3d(1,to)], ...
          [points_3d(2,from) points_3d(2,to)], ...
          [points_3d(3,from) points_3d(3,to)], ...
          'Color', color, 'LineWidth', 3);
end