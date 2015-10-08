%SHOWFEATURES Summary of this function goes here
%   Detailed explanation goes here
function showFeatures( I_0, keypoints_0, I_1, keypoints_1, map, all)

    if (nargin < 6)
        all = false
    end

    figure(1); clf;
    colormap(map);
    circle_size = 70;
    color = 'black';

    subplot(1, 2, 1);
    image(I_0);

    subplot(1, 2, 2);
    image(I_1);

    if all
        % draw keypoints all at once
        subplot(1, 2, 1);
        hold on;
        scatter(keypoints_0(1,:), keypoints_0(2,:), circle_size, color);

        subplot(1, 2, 2);
        hold on;
        scatter(keypoints_1(1,:), keypoints_1(2,:), circle_size, color);
    else
        % draw keypoints after klick
        for i = 1 : size(keypoints_0, 2)
            subplot(1, 2, 1);
            hold on;
            scatter(keypoints_0(1,i), keypoints_0(2,i), circle_size, color);

            subplot(1, 2, 2);
            hold on;
            scatter(keypoints_1(1,i), keypoints_1(2,i), circle_size, color);

            [x, y, btn] = ginput(1);

            if btn == 3
                return
            end
        end
    end
end

