
function draw_3D_points(est_points, gt_points, rmse, label, est_color)
    % Top view of estimated vs ground truth landmarks, with a grey line
    % between each landmark and its GT position
    if nargin < 4
        label = sprintf('landmarks_rmse%.3f', rmse);
    end
    if nargin < 5
        est_color = [1 0 0];   % default: red (pre-BA)
    end

    figure('Name', label, 'NumberTitle', 'off');
    hold on;
    grid on;
    axis equal;

    % Error lines first, so they stay behind the markers
    for k = 1:size(gt_points, 2)
        plot([gt_points(1,k), est_points(1,k)], ...
             [gt_points(2,k), est_points(2,k)], ...
             '-', 'Color', [0.7 0.7 0.7]);
    end

    % Ground Truth: blue circles
    if ~isempty(gt_points)
        h_gt = plot(gt_points(1,:), gt_points(2,:), ...
                    'bo', 'MarkerSize', 5, 'LineWidth', 1.5);
    end

    % Estimated: crosses in est_color
    if ~isempty(est_points)
        h_est = plot(est_points(1,:), est_points(2,:), ...
                     'x', 'Color', est_color, 'MarkerSize', 5, 'LineWidth', 1.5);
    end

    title(sprintf('%s  (RMSE: %.3fm)', strrep(label,'_',' '), rmse));
    xlabel('X (m)'); ylabel('Y (m)');
    if exist('h_gt', 'var') && exist('h_est', 'var')
        legend([h_gt, h_est], 'Ground Truth', 'Estimated', 'Location', 'northwest');
    end

    % Save figure to file
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
