%MANUALFEATUREDETECT Summary of this function goes here
%   Detailed explanation goes here

function [ keypoints_0, keypoints_1 ] = manualFeatureDetect( I_0, I_1, map)
    figure(1);
    
    if (nargin == 2)
        map = I_1;
    end
    
    colormap(map);
    
    done = false;
    idx = 1;
    keypoints_0 = [0; 0];
    keypoints_1 = [0; 0];

    while done == false        
        [x0, y0, done] = getKeypoints(I_0, keypoints_0);
      
        if done == true
            break;
        end
        
        if (nargin > 2)
            [x1, y1, done] = getKeypoints(I_1, keypoints_1);
        end
        
        if done == false;
            keypoints_0(:, idx) = [x0; y0];
            if (nargin > 2)
                keypoints_1(:, idx) = [x1; y1];
            end
            idx = idx + 1;
        end
    end
end

function [ x, y, done] = getKeypoints(img, prev_points)
    image(img);
    hold on;

    % draw all previous keypoints
    for i = 1 : size(prev_points, 2)
        scatter(prev_points(1,i), prev_points(2,i), 70, 'yellow');
    end

    [x, y, btn] = ginput(1);

    if btn == 3
        done = true;
    else
        done = false;
    end

    hold off;
end