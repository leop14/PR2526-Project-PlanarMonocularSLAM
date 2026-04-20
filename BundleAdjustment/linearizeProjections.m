# linearizes the robot-landmark measurements
function [H, b, chi_tot, num_inliers] = linearizeProjections(XR, XL, Zl, associations, num_poses, num_landmarks, kernel_threshold, img_width, img_height, K, T_cam_rob)
    global pose_dim;
    global landmark_dim;
    system_size = pose_dim*num_poses + landmark_dim*num_landmarks;
    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_tot = 0;
    num_inliers = 0;

    for (measurement_num = 1:size(Zl,2))
        pose_index = associations(1, measurement_num);
        landmark_index = associations(2, measurement_num);
        z = Zl(:, measurement_num);

        T_rob_world = XR(:, :, pose_index);
        p_land_world = XL(:, landmark_index);

        [is_valid, e, Jr, Jl] = projectionErrorAndJacobian(T_rob_world, p_land_world, z, img_width, img_height, K, T_cam_rob);

        if (! is_valid)
            continue;
        endif;

        chi = e'*e;
        if (chi > kernel_threshold)
            e *= sqrt(kernel_threshold/chi);
            chi = kernel_threshold;
        else
            num_inliers++;
        endif;
        chi_tot += chi;

        pose_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks);
        landmark_matrix_index = landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);

        H(pose_matrix_index:pose_matrix_index+pose_dim-1,
            pose_matrix_index:pose_matrix_index+pose_dim-1) += Jr'*Jr;

        H(pose_matrix_index:pose_matrix_index+pose_dim-1,
            landmark_matrix_index:landmark_matrix_index+landmark_dim-1) += Jr'*Jl;

        H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
            landmark_matrix_index:landmark_matrix_index+landmark_dim-1) += Jl'*Jl;

        H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
            pose_matrix_index:pose_matrix_index+pose_dim-1) += Jl'*Jr;

        b(pose_matrix_index:pose_matrix_index+pose_dim-1) += Jr'*e;
        b(landmark_matrix_index:landmark_matrix_index+landmark_dim-1) += Jl'*e;
    endfor
endfunction
