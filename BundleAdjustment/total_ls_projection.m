# Assembly of the projection problem
# Note: total_ls_indices.m must be sourced before this file
# (done in bundle_adjustment.m)


# dimension of projection
global projection_dim=2;


# projects a point
function p_img = projectPoint(Xr, Xl, img_width, img_height, K)
    iXr = inv(Xr);
    p_img = [-1;-1];
    pw = iXr(1:3,1:3)*Xl + iXr(1:3,4);
    if (pw(3) < 0)
        return;
    endif;
    p_cam = K*pw;
    iz = 1./p_cam(3);
    p_cam *= iz;
    if (p_cam(1) < 0 || p_cam(1) > img_width ||
        p_cam(2) < 0 || p_cam(2) > img_height)
        return;
    endif;
    p_img = p_cam(1:2);
endfunction

# error and jacobian of a measured landmark
# input:
#   T_rob_world: the robot pose in world frame (4x4 homogeneous matrix)
#   p_land_world: the landmark pose (3x1 vector, 3d position in world frame)
#   z:  projection of the landmark on the image plane
# output:
#   e: 2x1, the difference between prediction and measurement
#   Jr: 2x3 derivative w.r.t the error and a perturbation on the pose
#   Jl: 2x3 derivative w.r.t the error and a perturbation on the landmark
#   is_valid: true if projection ok

function [is_valid, e, Jr, Jl] = projectionErrorAndJacobian(T_rob_world, p_land_world, z, img_width, img_height, K, T_cam_rob)
    is_valid = false;
    e = [0;0];
    Jr = zeros(2,3);
    Jl = zeros(2,3);
    
    # Calculating full camera pose in the world
    T_cam_world = T_rob_world * T_cam_rob;
    iRc = T_cam_world(1:3,1:3)';
    itc = -iRc * T_cam_world(1:3,4);

    # point prediction, in world scale (CAMERA frame)
    p_cam_3d = iRc * p_land_world + itc; 
    if (p_cam_3d(3) < 0)
        return;
    endif

    # Calculating point prediction in ROBOT frame (for the Jacobian)
    iR_rob_world = T_rob_world(1:3,1:3)';
    itr = -iR_rob_world * T_rob_world(1:3,4);
    p_rob = iR_rob_world * p_land_world + itr;

    # Jwr: Derivative of the point in the robot frame w.r.t local SE(2) perturbation [dx, dy, dtheta]
    J_prob_pose = [-1,  0,  p_rob(2);
                    0, -1, -p_rob(1);
                    0,  0,  0];
    
    # Rotating the derivative into the camera frame
    iR_cam_offset = T_cam_rob(1:3,1:3)';
    Jwr = iR_cam_offset * J_prob_pose;

    Jwl = iRc;     # The landmark Jacobian is just the inverse rotation of the total camera pose

    # Projection on 2D image plane
    p_img_hom = K * p_cam_3d;
    iz = 1 ./ p_img_hom(3);
    z_hat = p_img_hom(1:2) * iz;

    if (z_hat(1) < 0 || z_hat(1) > img_width || 
            z_hat(2) < 0 || z_hat(2) > img_height)
        return;
    endif;

    iz2 = iz * iz;
    Jp = [iz, 0, -p_img_hom(1)*iz2;
            0,  iz, -p_img_hom(2)*iz2];
    
    e = z_hat - z;
    Jr = Jp * K * Jwr;  # Jr is naturally 2x3 now!
    Jl = Jp * K * Jwl;
    is_valid = true;

endfunction;


# linearizes the robot-landmark measurements
#   XR: the initial robot poses (4x4xnum_poses: array of homogeneous matrices)
#   XL: the initial landmark estimates (3xnum_landmarks matrix of landmarks)
#   Z:  the measurements (2xnum_measurements)
#   associations: 2xnum_measurements. 
#                 associations(:,k)=[p_idx,l_idx]' means the kth measurement
#                 refers to an observation made from pose p_idx, that
#                 observed landmark l_idx
#   num_poses: number of poses in XR (added for consistency)
#   num_landmarks: number of landmarks in XL (added for consistency)
#   kernel_threshod: robust kernel threshold
# output:
#   XR: the robot poses after optimization
#   XL: the landmarks after optimization
#   chi_stats: array 1:num_iterations, containing evolution of chi2
#   num_inliers: array 1:num_iterations, containing evolution of inliers

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
