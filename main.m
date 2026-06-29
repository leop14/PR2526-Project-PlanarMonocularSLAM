close all 
clear
clc 

addpath("data_read");
addpath("BundleAdjustment");

% "qt" renders via OpenGL/libGL, which has no GPU to talk to under WSL and
% silently produces empty figures. "gnuplot" renders straight to file
% without touching libGL/X11, so figure saving works headless.
graphics_toolkit("gnuplot");

cam_data = read_camera_data("data/camera.dat");


traj_data = read_traj_data("data/trajectory.dat");


draw_traj(traj_data, [], 'trajectory_initial');

meas_data_db = new_read_meas_data("data");

% Testing correctness: Verify a specific point (e.g. Point #6)
if isKey(meas_data_db, 6)
    pt = meas_data_db(6);
    fprintf('\nChecking Point ID #6\n');
    fprintf('\tObserved in %d frames.\n', pt.count);
    
    obs = pt.observations{1};
    fprintf('\tFirst obs: Frame %d at pixels [%.2f, %.2f]\n', ...
            obs.seq_num, obs.uv(1), obs.uv(2));
    fprintf('\tRobot Odom at that time: [%.4f, %.4f, %.4f]\n', obs.odom_pose);

    obs = pt.observations{pt.count};
    fprintf('\tLast obs: Frame %d at pixels [%.2f, %.2f]\n', ...
            obs.seq_num, obs.uv(1), obs.uv(2));
    fprintf('\tRobot Odom at that time: [%.4f, %.4f, %.4f]\n', obs.odom_pose);

end


world_gt_map = read_world_data("data/world.dat");


disp("Triangulating Points - method 1");
map_estimate = triangulate1(meas_data_db, cam_data.T, cam_data.K);

disp("Evaluating Map Quality");

% We want to compute the whole RMSE

squared_error_sum = 0;
count_evaluated = 0;

estimated_ids = cell2mat(keys(map_estimate));
gt_points = [];
est_points = [];

for i = 1:length(estimated_ids)
    id = estimated_ids(i);
    
    % Only evaluate if we have ground truth for this ID
    if isKey(world_gt_map, id)
        p_est = map_estimate(id);
        p_gt = world_gt_map(id);

        if isempty(p_est)
            continue;
        end
        
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
    rmse = sqrt(squared_error_sum / count_evaluated);
    fprintf('\n');
    fprintf('Map RMSE: %.4f meters\n', rmse);
    fprintf('Evaluated %d points.\n', count_evaluated);
else
    warning('No overlapping points found between Estimate and GT!');
end

%%%%%%%%%%
% Visualization 
%%%%%%%%%%

draw_3D_points(est_points, gt_points, rmse);



%%%%%%%%%%%%%
% Triang 2
%%%%%%%%%%%%%
disp("Triangulating Points - method 2");
map_estimate = triangulate2(meas_data_db, cam_data.T, cam_data.K);

disp("Preparing arrays for evaluation and solver...");
[XR_guess, XL_guess, Zr, pose_associations, Zp, projection_associations, landmark_ids_array, num_poses, num_landmarks] = ...
     prepare_solver_data(map_estimate, traj_data, meas_data_db);

disp("--- INITIAL TRAJECTORY (NEW) EVALUATION ---");
[trans_rmse_initial, rot_rmse_initial] = evaluate_traj(XR_guess, traj_data);

disp("\n--- INITIAL MAP EVALUATION (NEW) ---");
[map_rmse_initial, est_pts_initial, gt_pts_initial] = ...
             evaluate_map(XL_guess, landmark_ids_array, world_gt_map);



disp("\n\nEvaluating Map Quality (OLD)");
% We want to compute the whole RMSE
squared_error_sum = 0;
count_evaluated = 0;

estimated_ids = cell2mat(keys(map_estimate));
gt_points = [];
est_points = [];

for i = 1:length(estimated_ids)
    id = estimated_ids(i);
    
    % Only evaluate if we have ground truth for this ID
    if isKey(world_gt_map, id)
        p_est = map_estimate(id);
        p_gt = world_gt_map(id);

        if isempty(p_est)
            continue;
        end
        
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
    rmse = sqrt(squared_error_sum / count_evaluated);
    fprintf('\n');
    fprintf('Map RMSE: %.4f meters\n', rmse);
    fprintf('Evaluated %d points.\n', count_evaluated);
else
    warning('No overlapping points found between Estimate and GT!');
end

%%%%%%%%%%
% Visualization 
%%%%%%%%%%

draw_3D_points(est_points, gt_points, rmse);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BUNDLE ADJUSTMENT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath("BundleAdjustment");

disp('');
disp('=== Bundle Adjustment ===');

% Prepare solver arrays from triangulate2 result
[XR_guess, XL_guess, Zr, pose_associations, Zp, projection_associations, ...
 landmark_ids_array, num_poses, num_landmarks] = ...
    prepare_solver_data(map_estimate, traj_data, meas_data_db);

% Initial evaluation (odometry + triangulated map)
disp('--- Initial trajectory (odometry) ---');
evaluate_traj(XR_guess, traj_data);

disp('--- Initial map (triangulation 2) ---');
[rmse_map_initial, est_pts_initial, gt_pts_initial] = ...
    evaluate_map(XL_guess, landmark_ids_array, world_gt_map);

% Run Gauss-Newton BA
num_iterations        = 30;
kernel_threshold_proj = 1000;   % pixels^2
kernel_threshold_pose = 1.0;    % flattened-matrix units
pose_weight           = 1000;   % information weight on odometry term vs projection term

disp('Running Bundle Adjustment');
[XR_opt, XL_opt, chi_stats, num_inliers_stats] = bundle_adjustment( ...
    XR_guess, XL_guess, Zp, projection_associations, Zr, pose_associations, ...
    num_poses, num_landmarks, ...
    num_iterations, kernel_threshold_proj, kernel_threshold_pose, pose_weight, ...
    cam_data.width, cam_data.height, cam_data.K, cam_data.T);

% Final evaluation
disp('');
disp('--- Final trajectory (after BA) ---');
evaluate_traj(XR_opt, traj_data);
draw_traj(traj_data, XR_opt, 'trajectory_final');

disp('--- Final map (after BA) ---');
[rmse_map_ba, est_pts_ba, gt_pts_ba] = ...
    evaluate_map(XL_opt, landmark_ids_array, world_gt_map);

draw_3D_points(est_pts_ba, gt_pts_ba, rmse_map_ba);

% Convergence plot
figure;
subplot(2,1,1);
plot(1:num_iterations, chi_stats, 'b-o', 'LineWidth', 1.5);
xlabel('Iteration'); ylabel('chi2'); title('BA convergence: chi2'); grid on;
subplot(2,1,2);
plot(1:num_iterations, num_inliers_stats, 'r-o', 'LineWidth', 1.5);
xlabel('Iteration'); ylabel('Inliers'); title('BA convergence: inliers'); grid on;