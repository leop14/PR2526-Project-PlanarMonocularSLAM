
function draw_traj(traj_data)
    % Create a new figure
    figure(1); % Force figure 1
    clf; 
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

    % Add Start/End Markers 
    % Green Circle for Start
    plot(traj_data.gt(1,1), traj_data.gt(1,2), 'go', ...
         'MarkerFaceColor', 'g', 'MarkerSize', 8); 
    % Black Square for End
    plot(traj_data.gt(end,1), traj_data.gt(end,2), 'ks', ...
         'MarkerFaceColor', 'k', 'MarkerSize', 8);

    % Labels and Legend
    title('Robot Trajectory: Odometry vs Ground Truth');
    xlabel('X Position');
    ylabel('Y Position');
    
    legend([h_odom, h_gt], 'Odometry (Noisy)', 'Ground Truth', 'Location', 'northwest');

    % Force Draw - This ensures the graphic processes before the script ends/crashes
    drawnow;
    
    hold off;
end