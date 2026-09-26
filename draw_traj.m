
function draw_traj(traj_data, XR_est, save_name)
    % Plots Odometry vs Ground Truth, optionally overlaying an estimated
    % trajectory (e.g. the Bundle-Adjustment output XR_opt).
    if nargin < 3
        save_name = 'trajectory';
    end

    % Create a new figure
    figure(1); % Force figure 1
    clf;
    % Wide enough that the legend beside the axes doesn't squeeze the plot.
    % The saved size is fixed through the paper size (6 x 3.75 in at -r150 =
    % 900 x 562 px): figure 1 is reused for every trajectory, and its on-screen
    % size is not reliable, so the image size must not depend on it.
    set(gcf, 'Position', [100, 100, 900, 560]);
    set(gcf, 'PaperUnits', 'inches', 'PaperPositionMode', 'manual', ...
             'PaperPosition', [0, 0, 6, 3.75]);
    hold on;
    grid on;
    axis equal; % Essential to preserve real-world geometry

    % Plot Odometry
    % Red dashed line with small dots
    h_odom = plot(traj_data.odom(:,1), traj_data.odom(:,2), ...
                  'r--.', 'LineWidth', 1, 'MarkerSize', 8);

    % Plot Ground Truth (The "Reality")
    % Blue solid line
    h_gt = plot(traj_data.gt(:,1), traj_data.gt(:,2), ...
                'b-', 'LineWidth', 2);

    handles = [h_odom, h_gt];
    labels = {'Odometry', 'Ground Truth'};

    % Optionally overlay the estimated (e.g. post-BA) trajectory
    if nargin > 1 && ~isempty(XR_est)
        num_poses = size(XR_est, 3);
        est_xy = zeros(num_poses, 2);
        for i = 1:num_poses
            est_xy(i, :) = XR_est(1:2, 4, i)';
        end
        % Same style as the odometry (dashed line with dots), in green
        h_est = plot(est_xy(:,1), est_xy(:,2), 'g--.', 'LineWidth', 1, 'MarkerSize', 8);
        handles(end+1) = h_est;
        labels{end+1} = 'Estimated';
    end

    % Add Start/End Markers
    % Green Circle for Start
    plot(traj_data.gt(1,1), traj_data.gt(1,2), 'go', ...
         'MarkerFaceColor', 'g', 'MarkerSize', 8);
    % Black Square for End
    plot(traj_data.gt(end,1), traj_data.gt(end,2), 'ks', ...
         'MarkerFaceColor', 'k', 'MarkerSize', 8);

    % Labels and Legend
    title('Robot Trajectory');
    xlabel('X Position');
    ylabel('Y Position');

    % Outside the axes, in the side margin left free by 'axis equal',
    % so it never covers the trajectory or the start/end markers
    legend(handles, labels, 'Location', 'eastoutside', 'FontSize', 9);

    % Force Draw - This ensures the graphic processes before the script ends/crashes
    drawnow;

    % Save figure to file
    out_path = sprintf('figures/%s.png', save_name);
    try
        print(gcf, out_path, '-dpng', '-r150');
        fprintf('Figure saved to: %s\n', out_path);
    catch err
        fprintf('Warning: could not save figure (%s)\n', err.message);
    end

    hold off;
end
