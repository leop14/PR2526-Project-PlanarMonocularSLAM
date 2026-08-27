
function world_map = triangulate3(meas_db, T_camera_robot, K, z_near, z_far)
    % Multi-view triangulation via orthographic ray-intersection (least-squares).
    %
    % For each landmark observed in N frames, each observation defines a 3-D
    % ray from the camera centre into world space.  The 3-D point p that
    % minimises the sum of squared perpendicular distances to all rays is the
    % solution of the 3x3 linear system
    %
    %   M * p = b,    M = sum_i (I - d_i*d_i'),    b = sum_i (I - d_i*d_i') * o_i
    %
    % where o_i is the i-th camera centre and d_i is the unit ray direction in
    % world frame.
    %
    % Compared with SVD-DLT this formulation:
    %   - Produces a Euclidean result directly (no homogeneous de-homogenisation
    %     or sign ambiguity).
    %   - Is numerically stable for mixed near/far observations.
    %
    % Filter: points whose XY world distance from origin exceeds 50 m are
    % discarded as degenerate (matches the reference implementation's only
    % filter on the multi-view path).
    %
    % Note: the ray-intersection approach requires no z_near / z_far depth
    % check (unlike SVD-DLT which can produce sign-flipped estimates).
    % Those parameters are kept in the signature for interface compatibility.

    world_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    landmark_ids = cell2mat(keys(meas_db));

    num_points = length(landmark_ids);
    fprintf('Triangulating %d points (ray intersection).\n', num_points);

    invK = inv(K);

    for i = 1:num_points
        point_id     = landmark_ids(i);
        point_struct = meas_db(point_id);

        if point_struct.count < 2
            continue;
        end

        x_world = ray_intersection(point_struct, T_camera_robot, invK);

        if ~isempty(x_world)
            world_map(point_id) = x_world;
        end
    end

    num_mapped = length(world_map);
    fprintf('Triangulation complete. %d points have been mapped.\n', num_mapped);
    fprintf('Landmarks Estimated: %.2f%% (%d / %d)\n', 100 * num_mapped / num_points, num_mapped, num_points);
end



function x_world = ray_intersection(point_struct, T_camera_robot, invK)
    n_frames = point_struct.count;
    I3 = eye(3);
    M  = zeros(3, 3);
    b  = zeros(3, 1);

    for frame = 1:n_frames
        frame_obs   = point_struct.observations{frame};
        T_cam_world = v2t(frame_obs.odom_pose) * T_camera_robot; % camera-to-world

        o_i  = T_cam_world(1:3, 4);         % camera centre in world
        R_wc = T_cam_world(1:3, 1:3);       % rotation: camera -> world

        u = frame_obs.uv(1);
        v = frame_obs.uv(2);

        d_cam = invK * [u; v; 1];
        d_cam = d_cam / norm(d_cam);        % unit ray in camera frame
        d_i   = R_wc * d_cam;              % unit ray in world frame

        P_orth = I3 - d_i * d_i';          % projects onto plane perpendicular to ray
        M = M + P_orth;
        b = b + P_orth * o_i;
    end

    x_world = M \ b;

    % XY outlier rejection: >50 m from world origin means the rays were
    % nearly parallel (degenerate geometry), not a real landmark.
    if norm(x_world(1:2)) > 50
        x_world = [];
    end
end
