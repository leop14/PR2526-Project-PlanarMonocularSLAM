close all
clear
clc

addpath("data_read");
addpath("BundleAdjustment");
addpath("evaluation");
addpath("plotting");

% gnuplot instead of qt: under WSL qt needs OpenGL and saves empty figures
graphics_toolkit("gnuplot");

cam_data    = read_camera_data("data/camera.dat");
traj_data   = read_traj_data("data/trajectory.dat");
draw_traj(traj_data, [], 'trajectory_odometry');
meas_data_db = read_meas_data("data");

% Quick check that the measurements are read correctly (landmark 6)
if isKey(meas_data_db, 6)
    pt = meas_data_db(6);
    fprintf('\nChecking Point ID #6\n');
    fprintf('\tObserved in %d frames.\n', pt.count);
    obs = pt.observations{1};
    fprintf('\tFirst obs: Frame %d at pixels [%.2f, %.2f]\n', obs.seq_num, obs.uv(1), obs.uv(2));
    fprintf('\tRobot Odom at that time: [%.4f, %.4f, %.4f]\n', obs.odom_pose);
    obs = pt.observations{pt.count};
    fprintf('\tLast obs: Frame %d at pixels [%.2f, %.2f]\n', obs.seq_num, obs.uv(1), obs.uv(2));
    fprintf('\tRobot Odom at that time: [%.4f, %.4f, %.4f]\n', obs.odom_pose);
end

world_gt_map = read_world_data("data/world.dat");

% BA parameters, the same for all three methods
num_iterations        = 50;     % max iterations, BA usually stops earlier
convergence_tol       = 1e-5;   % stop when chi2 changes by less than this (relative)
kernel_threshold_proj = 1000;   % pixels^2
kernel_threshold_pose = 1.0;
pose_weight           = 1000;   % weight of odometry w.r.t. projections
outlier_threshold     = 0.5;    % [m] landmarks farther than this from GT are outliers

% chi2 and inliers per iteration, one row per method
chi_all = nan(3, num_iterations);
inl_all = nan(3, num_iterations);

% results for the summary table at the end
stats = struct('name', {'Method 1', 'Method 2', 'Method 3'});


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% METHOD 1: Consecutive-pairs DLT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('');
disp('=== Method 1: Consecutive-pairs DLT ===');
t_start = tic;
map1 = triangulate1(meas_data_db, cam_data.T, cam_data.K, cam_data.z_near, cam_data.z_far);
stats(1).t_tri = toc(t_start);

[XR1, XL1, Zr, pose_assoc, Zp1, proj_assoc1, ids1, num_poses, num_lm1] = ...
    prepare_solver_data(map1, traj_data, meas_data_db);

disp('--- Initial (odometry + triangulation 1) ---');
evaluate_traj(XR1, traj_data);
[rmse1_pre, ep1_pre, gp1_pre, err1_pre] = evaluate_map(XL1, ids1, world_gt_map, outlier_threshold);
draw_3D_points(ep1_pre, gp1_pre, rmse1_pre, 'landmarks_method1_pre_ba');

disp('Running Bundle Adjustment (method 1)');
t_start = tic;
[XR1_opt, XL1_opt, chi1, inl1, H1_initial, stats(1).iters] = bundle_adjustment( ...
    XR1, XL1, Zp1, proj_assoc1, Zr, pose_assoc, num_poses, num_lm1, ...
    num_iterations, kernel_threshold_proj, kernel_threshold_pose, pose_weight, ...
    cam_data.width, cam_data.height, cam_data.K, cam_data.T, convergence_tol);
stats(1).t_ba = toc(t_start);

disp('--- Final (after BA, method 1) ---');
evaluate_traj(XR1_opt, traj_data);
[rmse1_ba, ep1_ba, gp1_ba, err1_ba, eval_ids1] = evaluate_map(XL1_opt, ids1, world_gt_map, outlier_threshold);
draw_3D_points(ep1_ba, gp1_ba, rmse1_ba, 'landmarks_method1_post_ba', [0 0.55 0]);
draw_traj(traj_data, XR1_opt, 'trajectory_method1_post_ba');
chi_all(1, :) = chi1;
inl_all(1, :) = inl1;
stats(1).err_pre = err1_pre;
stats(1).err_ba  = err1_ba;
stats(1).ids     = eval_ids1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% METHOD 3: Ray intersection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('');
disp('=== Method 3: Ray intersection ===');
t_start = tic;
map3 = triangulate3(meas_data_db, cam_data.T, cam_data.K, cam_data.z_near, cam_data.z_far);
stats(3).t_tri = toc(t_start);

[XR3, XL3, Zr, pose_assoc, Zp3, proj_assoc3, ids3, num_poses, num_lm3] = ...
    prepare_solver_data(map3, traj_data, meas_data_db);

disp('--- Initial (odometry + triangulation 3) ---');
evaluate_traj(XR3, traj_data);
[rmse3_pre, ep3_pre, gp3_pre, err3_pre] = evaluate_map(XL3, ids3, world_gt_map, outlier_threshold);
draw_3D_points(ep3_pre, gp3_pre, rmse3_pre, 'landmarks_method3_pre_ba');

disp('Running Bundle Adjustment (method 3)');
t_start = tic;
[XR3_opt, XL3_opt, chi3, inl3, H3_initial, stats(3).iters] = bundle_adjustment( ...
    XR3, XL3, Zp3, proj_assoc3, Zr, pose_assoc, num_poses, num_lm3, ...
    num_iterations, kernel_threshold_proj, kernel_threshold_pose, pose_weight, ...
    cam_data.width, cam_data.height, cam_data.K, cam_data.T, convergence_tol);
stats(3).t_ba = toc(t_start);

disp('--- Final (after BA, method 3) ---');
evaluate_traj(XR3_opt, traj_data);
[rmse3_ba, ep3_ba, gp3_ba, err3_ba, eval_ids3] = evaluate_map(XL3_opt, ids3, world_gt_map, outlier_threshold);
draw_3D_points(ep3_ba, gp3_ba, rmse3_ba, 'landmarks_method3_post_ba', [0 0.55 0]);
draw_traj(traj_data, XR3_opt, 'trajectory_method3_post_ba');
chi_all(3, :) = chi3;
inl_all(3, :) = inl3;
stats(3).err_pre = err3_pre;
stats(3).err_ba  = err3_ba;
stats(3).ids     = eval_ids3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% METHOD 2: All-pairs DLT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('');
disp('=== Method 2: All-pairs DLT ===');
t_start = tic;
map2 = triangulate2(meas_data_db, cam_data.T, cam_data.K, cam_data.z_near, cam_data.z_far);
stats(2).t_tri = toc(t_start);

[XR2, XL2, Zr, pose_assoc, Zp2, proj_assoc2, ids2, num_poses, num_lm2] = ...
    prepare_solver_data(map2, traj_data, meas_data_db);

disp('--- Initial (odometry + triangulation 2) ---');
evaluate_traj(XR2, traj_data);
[rmse2_pre, ep2_pre, gp2_pre, err2_pre] = evaluate_map(XL2, ids2, world_gt_map, outlier_threshold);
draw_3D_points(ep2_pre, gp2_pre, rmse2_pre, 'landmarks_method2_pre_ba');

disp('Running Bundle Adjustment (method 2)');
t_start = tic;
[XR2_opt, XL2_opt, chi2, inl2, H2_initial, stats(2).iters] = bundle_adjustment( ...
    XR2, XL2, Zp2, proj_assoc2, Zr, pose_assoc, num_poses, num_lm2, ...
    num_iterations, kernel_threshold_proj, kernel_threshold_pose, pose_weight, ...
    cam_data.width, cam_data.height, cam_data.K, cam_data.T, convergence_tol);
stats(2).t_ba = toc(t_start);

disp('--- Final (after BA, method 2) ---');
evaluate_traj(XR2_opt, traj_data);
[rmse2_ba, ep2_ba, gp2_ba, err2_ba, eval_ids2] = evaluate_map(XL2_opt, ids2, world_gt_map, outlier_threshold);
draw_3D_points(ep2_ba, gp2_ba, rmse2_ba, 'landmarks_method2_post_ba', [0 0.55 0]);
draw_traj(traj_data, XR2_opt, 'trajectory_method2_post_ba');
chi_all(2, :) = chi2;
inl_all(2, :) = inl2;
stats(2).err_pre = err2_pre;
stats(2).err_ba  = err2_ba;
stats(2).ids     = eval_ids2;

% Sparsity of H at the first iteration, for each method
draw_H_sparsity(H1_initial, num_poses, num_lm1, 'method 1', 'H_sparsity_method1');
draw_H_sparsity(H2_initial, num_poses, num_lm2, 'method 2', 'H_sparsity_method2');
draw_H_sparsity(H3_initial, num_poses, num_lm3, 'method 3', 'H_sparsity_method3');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convergence comparison plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name', 'BA Convergence', 'NumberTitle', 'off');
last_iter = max([stats.iters]);
iters = 1:last_iter;

% log scale, otherwise after a few iterations all curves look flat at zero
subplot(2,1,1);
hold on; grid on;
plot(iters, chi_all(1, iters), 'b-o', 'LineWidth', 1.5);
plot(iters, chi_all(3, iters), 'g-s', 'LineWidth', 1.5);
plot(iters, chi_all(2, iters), 'r-^', 'LineWidth', 1.5);
set(gca, 'yscale', 'log');
xlim([1, last_iter]);
xlabel('Iteration'); ylabel('chi2 (proj + pose)');
title('BA convergence: total chi2 (log scale)');
legend('Method 1', 'Method 3', 'Method 2', 'Location', 'northeast');

subplot(2,1,2);
hold on; grid on;
plot(iters, inl_all(1, iters), 'b-o', 'LineWidth', 1.5);
plot(iters, inl_all(3, iters), 'g-s', 'LineWidth', 1.5);
plot(iters, inl_all(2, iters), 'r-^', 'LineWidth', 1.5);
xlim([1, last_iter]);
xlabel('Iteration'); ylabel('Inliers');
title('BA convergence: inliers');
legend('Method 1', 'Method 3', 'Method 2', 'Location', 'southeast');

try
    print(gcf, 'figures/ba_convergence.png', '-dpng', '-r150');
    fprintf('Figure saved to: figures/ba_convergence.png\n');
catch err
    fprintf('Warning: could not save figure (%s)\n', err.message);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Landmark error vs number of observations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for m = 1:3
    stats(m).num_obs = zeros(size(stats(m).ids));
    for k = 1:numel(stats(m).ids)
        entry = meas_data_db(int32(stats(m).ids(k)));
        stats(m).num_obs(k) = entry.count;
    end
end
draw_error_vs_obs({stats.err_ba}, {stats.num_obs}, ...
                  {'Method 1 (consecutive DLT)', 'Method 2 (all-pairs DLT)', 'Method 3 (ray intersection)'}, ...
                  [0 0 1; 1 0 0; 0 0.6 0], outlier_threshold, 'landmark_error_vs_obs');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Summary
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\n=== Summary (landmark errors in m, times in s) ===\n');
fprintf('%-9s | %9s %9s | %9s %9s %9s %9s %9s | %5s | %8s %8s %8s\n', ...
        'Method', 'RMSE pre', 'med pre', 'RMSE BA', 'med BA', 'p90 BA', 'max BA', ...
        sprintf('>%.1fm', outlier_threshold), 'iters', 't_tri', 't_BA', 't_BA/it');
for m = 1:3
    s = stats(m);
    fprintf('%-9s | %9.4f %9.4f | %9.4f %9.4f %9.4f %9.4f %9d | %5d | %8.1f %8.1f %8.1f\n', ...
            s.name, sqrt(mean(s.err_pre.^2)), median(s.err_pre), ...
            sqrt(mean(s.err_ba.^2)), median(s.err_ba), quantile(s.err_ba, 0.9), max(s.err_ba), ...
            sum(s.err_ba > outlier_threshold), s.iters, s.t_tri, s.t_ba, s.t_ba / s.iters);
end
