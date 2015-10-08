% VISUALIZE3DPOINTS Summary of this function goes here
%   Detailed explanation goes here

function [ ] = visualize3DPoints( fig, points_3d, repr_errors, color, color_rejected)

    reprojection_error_threshold = 30;

    if (nargin < 4)
       color = 'blue'; 
    end
    if (nargin < 5)
        color_rejected = 'red';
    end

    figure(fig);
    rotate3d on; %grid on;
    axis('equal'); axis vis3d; axis tight;
    view(50, 20);
    hold on;

    % plot origin
    %plot3(3*dX*[1 0 0 0 0], 3*dX*[0 0 1 0 0], 3*dX*[0 0 0 0 1], 'r-', 'linewidth', 3);
%     plot3(max(points_3d(1,:)) * [1 0 0 0 0], ...
%           max(points_3d(2,:)) * [0 0 1 0 0], ...
%           max(points_3d(3,:)) * [0 0 0 0 1], 'r-', 'linewidth', 3);
    %plot3([1 0 0 0 0], [0 0 1 0 0], [0 0 0 0 1], 'r-', 'linewidth', 3);
    
	if (nargin < 3)
        % draw all points
        scatter3(points_3d(1,:), points_3d(2,:), points_3d(3,:), ones(1, size(points_3d, 2)) * 100, color, 'filled')    
    else
      for i = 1 : size(points_3d, 2)
          c = color;
          if (repr_errors(i) > reprojection_error_threshold)
              c = color_rejected;
          end

          scatter3(points_3d(1,i), points_3d(2,i), points_3d(3,i), 100, c, 'filled')
      end
	end
    
    xlabel('X_{world}');
    ylabel('Y_{world}');
    zlabel('Z_{world}');
end

