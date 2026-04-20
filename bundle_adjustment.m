function [XR, XL, chi_stats, num_inliers_stats] = bundle_adjustment( ...
    XR, XL, Zl, associations, num_poses, num_landmarks, ...
    num_iterations, kernel_threshold, img_width, img_height, K, T_cam_rob)

    addpath("BundleAdjustment");

    global pose_dim = 3;
    global landmark_dim = 3;
    global projection_dim = 2;

    chi_stats = zeros(1, num_iterations);
    num_inliers_stats = zeros(1, num_iterations);

    system_size = pose_dim*num_poses + landmark_dim*num_landmarks;

    for iter = 1:num_iterations
        [H, b, chi_tot, num_inliers] = linearizeProjections( ...
            XR, XL, Zl, associations, num_poses, num_landmarks, ...
            kernel_threshold, img_width, img_height, K, T_cam_rob);

        chi_stats(iter) = chi_tot;
        num_inliers_stats(iter) = num_inliers;

        % Gauge fix: anchor first pose
        H(1:pose_dim, :) = 0;
        H(:, 1:pose_dim) = 0;
        H(1:pose_dim, 1:pose_dim) = eye(pose_dim);
        b(1:pose_dim) = 0;

        dx = -(H \ b);

        [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);

        printf("Iter %d/%d: chi2=%.4f, inliers=%d/%d\n", ...
               iter, num_iterations, chi_tot, num_inliers, size(Zl, 2));
        fflush(stdout);
    endfor
endfunction
