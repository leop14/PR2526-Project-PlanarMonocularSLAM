
function world_map = triangulate2(meas_db, T_camera_robot, K, z_near, z_far)
    % Multi-view pairwise triangulation (all-pairs averaging).
    % Tries every (i, j) pair of observations for each landmark.
    % Each pair is solved independently with SVD-DLT; pairs whose triangulated
    % point falls outside [z_near, z_far) in either camera, or further than
    % 50 m from the world XY origin, are discarded.  Valid pair estimates are
    % averaged.  Using all pairs rather than only consecutive frames gives
    % large-baseline combinations that are much better conditioned, which is
    % what the reference implementation does.

    world_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    landmark_ids = cell2mat(keys(meas_db));

    num_points = length(landmark_ids);
    fprintf('Triangulating %d points (all-pairs pairwise).\n', num_points);

    for i = 1:num_points
        point_id     = landmark_ids(i);
        point_struct = meas_db(point_id);

        if point_struct.count < 2
            continue;
        end

        candidates = [];
        n_obs = point_struct.count;

        % Precompute projection matrices once per observation (not per pair).
        P_cache = cell(n_obs, 1);
        for k = 1:n_obs
            P_cache{k} = projection_matrix(point_struct.observations{k}, T_camera_robot, K);
        end

        for a = 1:(n_obs - 1)
            for b = (a + 1):n_obs
                obs_a = point_struct.observations{a};
                obs_b = point_struct.observations{b};

                X = triangulate_pair(P_cache{a}, obs_a.uv, P_cache{b}, obs_b.uv, z_near, z_far);

                if ~isempty(X)
                    candidates(:, end+1) = X;
                end
            end
        end

        if ~isempty(candidates)
            world_map(point_id) = mean(candidates, 2);
        end
    end

    num_mapped = length(world_map);
    fprintf('Triangulation complete. %d points have been mapped.\n', num_mapped);
    fprintf('Landmarks Estimated: %.2f%% (%d / %d)\n', 100 * num_mapped / num_points, num_mapped, num_points);
end



function X = triangulate_pair(P1, uv1, P2, uv2, z_near, z_far)
    u1 = uv1(1); v1 = uv1(2);
    u2 = uv2(1); v2 = uv2(2);

    A = [u1*P1(3,:) - P1(1,:);
         v1*P1(3,:) - P1(2,:);
         u2*P2(3,:) - P2(1,:);
         v2*P2(3,:) - P2(2,:)];

    [~, ~, V] = svd(A);
    X_hom = V(:, end);
    X = X_hom(1:3) / X_hom(4);

    % Depth filter: both cameras must see the point in [z_near, z_far).
    % Use the de-homogenized X to avoid sign ambiguity on X_hom.
    d1 = P1(3, 1:3) * X + P1(3, 4);
    d2 = P2(3, 1:3) * X + P2(3, 4);
    if d1 < z_near || d1 >= z_far || d2 < z_near || d2 >= z_far
        X = [];
        return;
    end

    % XY outlier rejection.
    if norm(X(1:2)) > 50
        X = [];
    end
end



function P = projection_matrix(obs, T_camera_robot, K)
    T_cam_world  = v2t(obs.odom_pose) * T_camera_robot; % camera-to-world
    T_world_cam  = inv(T_cam_world);                     % world-to-camera
    P = K * T_world_cam(1:3, :);
end
