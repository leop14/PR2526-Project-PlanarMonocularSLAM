# linearizes the pose-pose (odometry) measurements
# pose_weight multiplies the odometry contribution to H and b: the projection
# chi2 is orders of magnitude larger, so without it the odometry would have
# basically no effect
function [H, b, chi_tot, num_inliers] = linearizePoses(XR, XL, Zr, associations, num_poses, num_landmarks, kernel_threshold, pose_weight)
    global pose_dim;
    global landmark_dim;
    system_size = pose_dim*num_poses + landmark_dim*num_landmarks;
    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_tot = 0;
    num_inliers = 0;

    for (measurement_num = 1:size(Zr,2))
        pose_i_index = associations(1, measurement_num);
        pose_j_index = associations(2, measurement_num);
        Z = v2t(Zr(:, measurement_num));

        Xi = XR(:, :, pose_i_index);
        Xj = XR(:, :, pose_j_index);

        [e, Ji, Jj] = poseErrorAndJacobian(Xi, Xj, Z);

        chi = e'*e;
        if (chi > kernel_threshold)
            e *= sqrt(kernel_threshold/chi);
            chi = kernel_threshold;
        else
            num_inliers++;
        endif;
        chi_tot += chi;

        % weight only on H and b, the printed chi2 is the unweighted one
        Ji_w = sqrt(pose_weight) * Ji;
        Jj_w = sqrt(pose_weight) * Jj;
        e_w  = sqrt(pose_weight) * e;

        pose_i_matrix_index = poseMatrixIndex(pose_i_index, num_poses, num_landmarks);
        pose_j_matrix_index = poseMatrixIndex(pose_j_index, num_poses, num_landmarks);

        H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1,
            pose_i_matrix_index:pose_i_matrix_index+pose_dim-1) += Ji_w'*Ji_w;

        H(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1,
            pose_j_matrix_index:pose_j_matrix_index+pose_dim-1) += Ji_w'*Jj_w;

        H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1,
            pose_j_matrix_index:pose_j_matrix_index+pose_dim-1) += Jj_w'*Jj_w;

        H(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1,
            pose_i_matrix_index:pose_i_matrix_index+pose_dim-1) += Jj_w'*Ji_w;

        b(pose_i_matrix_index:pose_i_matrix_index+pose_dim-1) += Ji_w'*e_w;
        b(pose_j_matrix_index:pose_j_matrix_index+pose_dim-1) += Jj_w'*e_w;
    endfor
endfunction
