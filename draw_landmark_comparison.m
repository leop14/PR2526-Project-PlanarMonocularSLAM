
function draw_landmark_comparison(gt_points, pre_ba_points, post_ba_points, rmse_pre, rmse_ba, label)
    % Shows GT (blue circles), pre-BA estimates (red crosses), post-BA estimates
    % (green crosses) in the same XY plot.  Error lines connect GT to post-BA.

    if nargin < 6
        label = sprintf('landmarks_comparison_rmse%.3f', rmse_ba);
    end

    figure('Name', label, 'NumberTitle', 'off');
    hold on;
    grid on;
    axis equal;

    % Error lines from GT to post-BA (rendered behind markers)
    for k = 1:size(gt_points, 2)
        plot([gt_points(1,k), post_ba_points(1,k)], ...
             [gt_points(2,k), post_ba_points(2,k)], ...
             '-', 'Color', [0.75 0.75 0.75]);
    end

    % Ground Truth: blue circles
    h_gt = plot(gt_points(1,:), gt_points(2,:), ...
                'bo', 'MarkerSize', 5, 'LineWidth', 1.5);

    % Pre-BA estimates: red crosses
    h_pre = plot(pre_ba_points(1,:), pre_ba_points(2,:), ...
                 'rx', 'MarkerSize', 4, 'LineWidth', 1.0);

    % Post-BA estimates: green crosses
    h_post = plot(post_ba_points(1,:), post_ba_points(2,:), ...
                  'gx', 'MarkerSize', 4, 'LineWidth', 1.0);

    title(sprintf('%s  (pre-BA RMSE: %.3fm  post-BA RMSE: %.3fm)', ...
                  strrep(label, '_', ' '), rmse_pre, rmse_ba));
    xlabel('X (m)'); ylabel('Y (m)');
    legend([h_gt, h_pre, h_post], ...
           {'Ground Truth', 'Pre-BA', 'Post-BA'}, 'Location', 'northwest');

    if ~exist('figures', 'dir')
        mkdir('figures');
    end
    filename = sprintf('figures/%s.png', label);
    try
        print(gcf, filename, '-dpng', '-r150');
        fprintf('Figure saved to: %s\n', filename);
    catch err
        fprintf('Warning: could not save figure (%s)\n', err.message);
    end

    drawnow;
end
