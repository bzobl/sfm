% calculate the average distance between points
function [ avg_dist ] = rubikAverageDistance(pts, errors, error_limit, verbose)

    if (nargin < 3)
        error_limit = 5;
    end
    if (nargin < 4)
        verbose = true;
    end

    avgs = [];
    avgs_idx = 1;

    %red side
    [avgs, avgs_idx] = avgs_side(pts, errors, avgs, avgs_idx, 0, 6, 5, 5, 5, error_limit);
    %yellow side
    [avgs, avgs_idx] = avgs_side(pts, errors, avgs, avgs_idx, 30, 6, 6, 5, 5, error_limit);
    %blue side
    [avgs, avgs_idx] = avgs_side(pts, errors, avgs, avgs_idx, 66, 5, 5, 4, 4, error_limit);
    
    % blue -> red
    for i = 0 : 4
        pt1 = 71 + 5 * i;
        pt2 = 1 + 6*i;
        if (errors(pt1) >= error_limit) || (errors(pt2) >= error_limit)
        	continue;
        end

        avgs(avgs_idx) = calcAverage(pts, pt1, pt2);
        avgs_idx = avgs_idx + 1;
    end
    
    % blue -> yellow
    for i = 0 : 4
        pt1 = 91 - i;
        pt2 = 37 + 6*i;
        if (errors(pt1) >= error_limit) || (errors(pt2) >= error_limit)
        	continue;
        end

        avgs(avgs_idx) = calcAverage(pts, pt1, pt2);
        avgs_idx = avgs_idx + 1;
    end

    avg_dist = mean(avgs);
    
    if verbose
        fprintf('Error threshold was %f. Did %d measurements for %d points\n', error_limit, avgs_idx - 1, size(pts, 2));
    end

end

function [ avgs, avgs_idx ] = avgs_side(pts, errors, avgs, avgs_idx, start, rows, cols, h_limit, v_limit, error_limit)
    for c = 0 : (cols - 1)
        for r = 0 : (rows - 1)
            pt_idx = start + (c * rows + r + 1);
            pt_idx_next_v = pt_idx + 1;
            pt_idx_next_h = pt_idx + rows;
            
            if (errors(pt_idx) >= error_limit)
                continue;
            end

            if (r < v_limit) && (errors(pt_idx_next_v) < error_limit)
                % vertical
                avgs(avgs_idx) = calcAverage(pts, pt_idx, pt_idx_next_v);
                avgs_idx = avgs_idx + 1; 
            end

            if (c < h_limit) && (errors(pt_idx_next_h) < error_limit)
                % horizontal
                avgs(avgs_idx) = calcAverage(pts, pt_idx, pt_idx_next_h);
                avgs_idx = avgs_idx + 1;     
            end
        end
    end
end

function [ avg ] = calcAverage(pts, idx1, idx2)
    plot3([ pts(1,idx1) pts(1,idx2) ], [ pts(2,idx1) pts(2,idx2) ], [ pts(3,idx1) pts(3,idx2) ]);
    avg = pdist([ pts(:,idx1) pts(:, idx2) ]', 'euclidean');
end