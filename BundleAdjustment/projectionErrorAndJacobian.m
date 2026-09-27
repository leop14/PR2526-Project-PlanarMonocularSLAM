# error and jacobian of a measured landmark
function [is_valid, e, Jr, Jl] = projectionErrorAndJacobian(T_rob_world, p_land_world, z, img_width, img_height, K, T_cam_rob)
    is_valid = false;
    e = [0;0];
    Jr = zeros(2,3);
    Jl = zeros(2,3);

    # Calculating full camera pose in the world
    T_cam_world = T_rob_world * T_cam_rob;
    iRc = T_cam_world(1:3,1:3)';
    itc = -iRc * T_cam_world(1:3,4);

    # landmark in the camera frame (skip it if it is behind the camera)
    p_cam_3d = iRc * p_land_world + itc;
    if (p_cam_3d(3) < 0)
        return;
    endif

    # Calculating point prediction in ROBOT frame (for the Jacobian)
    iR_rob_world = T_rob_world(1:3,1:3)';
    itr = -iR_rob_world * T_rob_world(1:3,4);
    p_rob = iR_rob_world * p_land_world + itr;

    # derivative of the point in the robot frame w.r.t. the local SE(2) perturbation [dx, dy, dtheta]
    J_prob_pose = [-1,  0,  p_rob(2);
                    0, -1, -p_rob(1);
                    0,  0,  0];

    # Rotating the derivative into the camera frame
    iR_cam_offset = T_cam_rob(1:3,1:3)';
    Jwr = iR_cam_offset * J_prob_pose;

    Jwl = iRc;     # for the landmark it's just the world -> camera rotation

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
    Jr = Jp * K * Jwr;
    Jl = Jp * K * Jwl;
    is_valid = true;
endfunction
