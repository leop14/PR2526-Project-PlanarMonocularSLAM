
function draw_traj(traj_data, XR_est, save_name)
    % Odometry vs ground truth, plus the estimated trajectory if XR_est
    % is given (e.g. after BA)
    if nargin < 3
        save_name = 'trajectory';
    end

    figure(1);
    clf;
    % Fixed image size (6 x 3.75 in at 150 dpi = 900 x 562 px). Figure 1 is
    % reused for every trajectory and its window size is not reliable, so
    % without this some images came out smaller.
    set(gcf, 'Position', [100, 100, 900, 560]);
    set(gcf, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', ...
             'PaperPosition', [0, 0, 6, 3.75]);
    hold on;
    grid on;
    axis equal;

    % Odometry: red dashed line with dots
    h_odom = plot(traj_data.odom(:,1), traj_data.odom(:,2), ...
                  'r--.', 'LineWidth', 1, 'MarkerSize', 8);

    % Ground truth: blue solid line
    h_gt = plot(traj_data.gt(:,1), traj_data.gt(:,2), ...
                'b-', 'LineWidth', 2);

    handles = [h_odom, h_gt];
    labels = {'Odometry', 'Ground Truth'};

    % Estimated trajectory: same style as the odometry, in green
    if nargin > 1 && ~isempty(XR_est)
        num_poses = size(XR_est, 3);
        est_xy = zeros(num_poses, 2);
        for i = 1:num_poses
            est_xy(i, :) = XR_est(1:2, 4, i)';
        end
        h_est = plot(est_xy(:,1), est_xy(:,2), 'g--.', 'LineWidth', 1, 'MarkerSize', 8);
        handles(end+1) = h_est;
        labels{end+1} = 'Estimated';
    end

    % Start (green circle) and end (black square)
    plot(traj_data.gt(1,1), traj_data.gt(1,2), 'go', ...
         'MarkerFaceColor', 'g', 'MarkerSize', 8);
    plot(traj_data.gt(end,1), traj_data.gt(end,2), 'ks', ...
         'MarkerFaceColor', 'k', 'MarkerSize', 8);

    title('Robot Trajectory');
    xlabel('X Position');
    ylabel('Y Position');

    % Legend outside the axes so it doesn't cover the trajectory
    legend(handles, labels, 'Location', 'eastoutside', 'FontSize', 9);

    drawnow;

    out_path = sprintf('figures/%s.png', save_name);
    try
        print(gcf, out_path, '-dpng', '-r150');
        fprintf('Figure saved to: %s\n', out_path);
    catch err
        fprintf('Warning: could not save figure (%s)\n', err.message);
    end

    hold off;
end
