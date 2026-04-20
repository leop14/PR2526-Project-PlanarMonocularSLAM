
function [rmse_opt, est_points, gt_points] = evaluate_map(XL_opt, landmark_ids_array, world_gt_map)
    % Evaluates a dense 3xN map array against the ground truth dictionary
    
    squared_error_sum = 0;
    count_evaluated = 0;
    num_landmarks = size(XL_opt, 2);
    
    gt_points = [];
    est_points = [];

    for col = 1:num_landmarks
        id = landmark_ids_array(col);
        
        % Only evaluate if we have ground truth for this ID
        if isKey(world_gt_map, id)
            p_est = XL_opt(:, col); 
            p_gt = world_gt_map(id);

            % Accumulate error
            diff = p_est - p_gt;
            squared_error_sum = squared_error_sum + sum(diff.^2);
            count_evaluated = count_evaluated + 1;
            
            % Store for plotting
            gt_points(:, end+1) = p_gt;
            est_points(:, end+1) = p_est;
        end
    end

    if count_evaluated > 0
        rmse_opt = sqrt(squared_error_sum / count_evaluated);
        fprintf('Optimized Map RMSE: %.4f meters\n', rmse_opt);
        fprintf('Evaluated %d points.\n', count_evaluated);
    else
        rmse_opt = -1;
        warning('No overlapping points found between Estimate and GT!');
    end
end