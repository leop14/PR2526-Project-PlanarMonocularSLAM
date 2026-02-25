
function draw_3D_points(est_points, gt_points, rmse)
    figure('Name', 'Map Comparison', 'NumberTitle', 'off');
    hold on; 
    grid on; 
    axis equal;
    view(3); % Set 3D view

    % Plot Ground Truth (Green Circles)
    if ~isempty(gt_points)
        h_gt = plot3(gt_points(1,:), gt_points(2,:), gt_points(3,:), ...
                    'go', 'MarkerSize', 5, 'LineWidth', 1.5);
    end

    % Plot Estimated (Red Crosses)
    if ~isempty(est_points)
        h_est = plot3(est_points(1,:), est_points(2,:), est_points(3,:), ...
                    'rx', 'MarkerSize', 6, 'LineWidth', 1.5);
    end

    % Draw lines connecting GT to Estimate (Visualizing Error)
    % -> helps to see exactly which point drifted where
    for k = 1:size(gt_points, 2)
        plot3([gt_points(1,k), est_points(1,k)], ...
            [gt_points(2,k), est_points(2,k)], ...
            [gt_points(3,k), est_points(3,k)], 'k-', 'Color', [0.7 0.7 0.7]);
    end

    title(sprintf('Map Initialization (RMSE: %.3fm)', rmse));
    xlabel('X'); ylabel('Y'); zlabel('Z');
    if exist('h_gt', 'var') && exist('h_est', 'var')
        legend([h_gt, h_est], 'Ground Truth', 'Estimated', 'Location', 'northwest');
    end

    disp('Visualization open. Press any key to exit');
    pause;