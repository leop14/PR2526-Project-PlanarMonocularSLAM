
function world_map = triangulate1(meas_db, T_camera_robot, K, z_near, z_far)
    % Method 1: for each landmark, triangulate every pair of consecutive
    % observations with DLT, then average the valid estimates.

    % Initializing the output map (ID -> [x; y; z])
    world_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');

    % Getting all keys (i.e. Landmark IDs) from the database
    landmark_ids = cell2mat(keys(meas_db));

    num_points = length(landmark_ids);
    fprintf('Triangulating %d points (pairwise).\n', num_points);

    for i = 1:num_points     % Iteration over all the points
        point_id = landmark_ids(i);
        point_struct = meas_db(point_id);

        if point_struct.count < 2
            continue;
        end

        % Collecting all "duplicate" estimates for this single landmark
        candidates = [];

        % We iterate through consecutive pairs (Frame 1-2, Frame 2-3, etc.)
        for k = 1:(point_struct.count - 1)
            obs1 = point_struct.observations{k};
            obs2 = point_struct.observations{k+1};

            % Attempt to triangulate this specific pair
            X_candidate = single_point_triangulation(obs1, obs2, T_camera_robot, K, z_near, z_far);

            % empty means the pair was rejected
            if ~isempty(X_candidate)
                candidates(:, end+1) = X_candidate;
            end
        end

        % Average of all the pair estimates
        if ~isempty(candidates)
            X_final = mean(candidates, 2);
            world_map(point_id) = X_final;
        end
    end

    num_mapped = length(world_map);
    fprintf('Pairwise Triangulation complete. %d points mapped.\n', num_mapped);
    fprintf('Landmarks Estimated: %.2f%% (%d / %d)\n', 100 * num_mapped / num_points, num_mapped, num_points);
end



function X = single_point_triangulation(obs1, obs2, T_camera_robot, K, z_near, z_far)
    % Helper to triangulate a point using two views

    % Get Projection Matrices P1 and P2
    P1 = get_projection_matrix(obs1, T_camera_robot, K);
    P2 = get_projection_matrix(obs2, T_camera_robot, K);

    % Linear system A*X = 0 (4x4, two rows per view):
    % u1 * (P1_row3 * X) - (P1_row1 * X) = 0
    % v1 * (P1_row3 * X) - (P1_row2 * X) = 0
    % ...
    A = zeros(4, 4);

    u1 = obs1.uv(1); v1 = obs1.uv(2);
    u2 = obs2.uv(1); v2 = obs2.uv(2);

    A(1, :) = u1 * P1(3, :) - P1(1, :);
    A(2, :) = v1 * P1(3, :) - P1(2, :);
    A(3, :) = u2 * P2(3, :) - P2(1, :);
    A(4, :) = v2 * P2(3, :) - P2(2, :);

    % Solve AX=0 system with SVD
    [~, ~, V] = svd(A);
    X_hom = V(:, end);
    X = X_hom(1:3) / X_hom(4);

    % Reject the point if its depth in one of the two cameras is outside
    % [z_near, z_far). We use the dehomogenized X, since the sign of the
    % SVD solution is arbitrary.
    depth1 = X_cam_depth(P1, [X; 1]);
    depth2 = X_cam_depth(P2, [X; 1]);
    if ~in_range(depth1, z_near, z_far) || ~in_range(depth2, z_near, z_far)
        X = [];
        return;
    end

    % Too far from the origin: rays almost parallel, not a real point
    if norm(X(1:2)) > 50
        X = [];
    end

end

function P = get_projection_matrix(obs, T_camera_robot, K)
    % Helper to build P = K * [R|t]
    T_robot_world = v2t(obs.odom_pose);
    T_camera_world = T_robot_world * T_camera_robot;
    T_world_camera = inv(T_camera_world); % The view matrix

    P = K * T_world_camera(1:3, :);
end

function depth = X_cam_depth(P, X_hom)
    % K's last row is [0 0 1], so the third component is the depth
    % in the camera frame
    p_cam = P * X_hom;
    depth = p_cam(3);
end

function ok = in_range(value, lo, hi)
    ok = (value >= lo) && (value < hi);
end
