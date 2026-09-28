
function [rmse_opt, est_points, gt_points, errors, eval_ids] = evaluate_map(XL_opt, landmark_ids_array, world_gt_map, outlier_threshold)
    % Compares the estimated landmarks (3xN) with the ground truth.
    % errors(k) is the distance from GT of the landmark with ID eval_ids(k)
    % (same order as the columns of est_points and gt_points).
    % Landmarks with error above outlier_threshold are counted as outliers.

    if nargin < 4
        outlier_threshold = 0.5;   % meters
    end

    num_landmarks = size(XL_opt, 2);

    gt_points = [];
    est_points = [];
    errors = [];
    eval_ids = [];

    for col = 1:num_landmarks
        id = landmark_ids_array(col);

        % Only evaluate if we have ground truth for this ID
        if isKey(world_gt_map, id)
            p_est = XL_opt(:, col);
            p_gt = world_gt_map(id);

            errors(end+1) = norm(p_est - p_gt);
            eval_ids(end+1) = id;

            % Store for plotting
            gt_points(:, end+1) = p_gt;
            est_points(:, end+1) = p_est;
        end
    end

    if ~isempty(errors)
        rmse_opt = sqrt(mean(errors.^2));
        fprintf('Map RMSE: %.4f meters\n', rmse_opt);
        fprintf('Evaluated %d points.\n', numel(errors));
        fprintf('Error median: %.4f m | p90: %.4f m | max: %.4f m | > %.2f m: %d\n', ...
                median(errors), quantile(errors, 0.9), max(errors), ...
                outlier_threshold, sum(errors > outlier_threshold));
    else
        rmse_opt = -1;
        warning('No overlapping points found between Estimate and GT!');
    end
end
