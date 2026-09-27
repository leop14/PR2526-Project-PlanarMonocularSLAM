function [XR, XL, chi_stats, num_inliers_stats, H_initial, num_iters_run] = bundle_adjustment( ...
    XR, XL, Zl, proj_associations, Zr, pose_associations, ...
    num_poses, num_landmarks, ...
    num_iterations, kernel_threshold_proj, kernel_threshold_pose, pose_weight, ...
    img_width, img_height, K, T_cam_rob, convergence_tol)
    % Total least squares on poses (SE(2)) and landmarks (R^3).
    % Runs at most num_iterations, but stops as soon as chi2 changes by less
    % than convergence_tol (relative). chi_stats / num_inliers_stats stay NaN
    % for the iterations that were not run.

    addpath("BundleAdjustment");

    global pose_dim = 3;
    global landmark_dim = 3;

    chi_stats = nan(1, num_iterations);
    num_inliers_stats = nan(1, num_iterations);
    H_initial = [];
    num_iters_run = num_iterations;

    damping = 0.01;

    for iter = 1:num_iterations
        % Projection factors
        [H_p, b_p, chi_p, inl_p] = linearizeProjections( ...
            XR, XL, Zl, proj_associations, num_poses, num_landmarks, ...
            kernel_threshold_proj, img_width, img_height, K, T_cam_rob);

        % Odometry (pose-pose) factors
        [H_r, b_r, chi_r, inl_r] = linearizePoses( ...
            XR, XL, Zr, pose_associations, num_poses, num_landmarks, ...
            kernel_threshold_pose, pose_weight);

        H = H_p + H_r;
        b = b_p + b_r;

        if iter == 1 && nargout >= 5
            H_initial = H;
        end

        chi_stats(iter) = chi_p + chi_r;
        num_inliers_stats(iter) = inl_p + inl_r;

        % chi2 is computed on the state after the previous update: if it
        % barely changed, that update did nothing and we can stop here
        if iter > 1 && abs(chi_stats(iter-1) - chi_stats(iter)) < convergence_tol * chi_stats(iter-1)
            num_iters_run = iter;
            printf("Converged at iter %d/%d: chi2_proj=%.2f (inl %d/%d)  chi2_pose=%.4f (inl %d/%d)\n", ...
                   iter, num_iterations, ...
                   chi_p, inl_p, size(Zl, 2), ...
                   chi_r, inl_r, size(Zr, 2));
            fflush(stdout);
            break;
        endif

        % damping, helps with badly conditioned landmarks (short baselines)
        H = H + damping * eye(size(H));

        % fix the gauge by anchoring the first pose
        H(1:pose_dim, :) = 0;
        H(:, 1:pose_dim) = 0;
        H(1:pose_dim, 1:pose_dim) = eye(pose_dim);
        b(1:pose_dim) = 0;

        dx = -(H \ b);

        [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);

        printf("Iter %d/%d: chi2_proj=%.2f (inl %d/%d)  chi2_pose=%.4f (inl %d/%d)\n", ...
               iter, num_iterations, ...
               chi_p, inl_p, size(Zl, 2), ...
               chi_r, inl_r, size(Zr, 2));
        fflush(stdout);
    endfor
endfunction
