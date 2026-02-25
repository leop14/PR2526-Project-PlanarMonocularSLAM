
function world_map = triangulate1(meas_db, T_camera_robot, K)
    % Triangulating pairwise:
    % i.   Extracting pairs of observations of same ID point
    % ii.  Triangulating each pair independently
    % iii. Merging the duplicate 3D points for the same ID

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
            X_candidate = single_point_triangulation(obs1, obs2, T_camera_robot, K);
            
            % Only keep valid results (not empty, not infinite)
            if ~isempty(X_candidate)
                candidates(:, end+1) = X_candidate;
            end
        end
        
        % Merging duplicates (Average the position)
        if ~isempty(candidates)
            % Compute centroid (mean along rows)
            X_final = mean(candidates, 2);
            world_map(point_id) = X_final;
        end
    end
    
    fprintf('Pairwise Triangulation complete. %d points mapped.\n', length(world_map));
end
    


function X = single_point_triangulation(obs1, obs2, T_camera_robot, K)
    % Helper to triangulate a point using two views
    
    % Get Projection Matrices P1 and P2
    P1 = get_projection_matrix(obs1, T_camera_robot, K);
    P2 = get_projection_matrix(obs2, T_camera_robot, K);
    
    % Formulating Linear System (4x4 matrix for 2 vies)
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
    
    % Check if point behind camera
    % Project back to camera 1 to check Z-depth
    X_cam1 = (P1 * X_hom); 
    if X_cam1(3) <= 0
        X = []; % Invalid point (behind camera)
    end
end

function P = get_projection_matrix(obs, T_camera_robot, K)
    % Helper to build P = K * [R|t]
    T_robot_world = v2t(obs.odom_pose);
    T_camera_world = T_robot_world * T_camera_robot;
    T_world_camera = inv(T_camera_world); % The view matrix
    
    P = K * T_world_camera(1:3, :);
end

function T = v2t(pose)
    % Converts [x, y, theta] to a 4x4 homogeneous matrix
    tx = pose(1);
    ty = pose(2);
    theta = pose(3);

    c = cos(theta);
    s = sin(theta);

    % 4x4 Matrix (Planar motion z=0)
    T = [c, -s, 0, tx;
         s,  c, 0, ty;
         0,  0, 1, 0;
         0,  0, 0, 1];
end