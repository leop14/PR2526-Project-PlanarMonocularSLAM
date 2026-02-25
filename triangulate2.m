
function world_map = triangulate2(meas_db, T_camera_robot, K)

    % Initializing the output map (ID -> [x; y; z])
    world_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');

    % Getting all keys (i.e. Landmark IDs) from the database
    landmark_ids = cell2mat(keys(meas_db));

    fprintf('Triangulating %d points.\n', length(landmark_ids));

    num_points = length(landmark_ids);
    for i = 1:num_points     % Iteration over all the points
        point_id = landmark_ids(i);
        point_struct = meas_db(point_id);

        if point_struct.count < 2
            continue;
        end
        
        % Perform triangulation on the current point
        x_world = single_point_triangulation(point_struct, T_camera_robot, K);

        % Store the result (Euclidean 3D point)
        world_map(point_id) = x_world;
    end
    
    fprintf('Triangulation complete. %d points have been mapped.\n', length(world_map));
end



function x_world = single_point_triangulation(point_struct, T_camera_robot, K)
    n_frames = point_struct.count;

    % Initializing Matrix A for the linear system A*x = 0
    A = zeros(2 * n_frames, 4);
    % Each frame provides 2 equations, so A is (2*N x 4)

    for frame = 1:n_frames
        frame_obs = point_struct.observations{frame};
        frame_odom_pose = frame_obs.odom_pose;
        T_robot_world = v2t(frame_odom_pose);

        T_camera_world = T_robot_world * T_camera_robot;
        T_world_camera = inv(T_camera_world);

        % Building Projection Matrix P (3x4)
        % P = K * [R | t], with R and t extracted from "T_world_camera"
        P = K * T_world_camera(1:3, :);

        % Formulating triangulation equations
        u = frame_obs.uv(1);
        v = frame_obs.uv(2);    

        row_idx = 2 * frame - 1;
        
        % Row 1: x-coordinate constraint
        A(row_idx, :)     = u * P(3, :) - P(1, :);
        
        % Row 2: y-coordinate constraint
        A(row_idx + 1, :) = v * P(3, :) - P(2, :);    
    end

    % Solve SVD (Total Least Squares)
    % We want the vector x that minimizes ||Ax|| subject to ||x||=1
    [~, ~, V] = svd(A);
    
    % The solution is the eigenvector corresponding to the smallest eigenvalue
    % This is the LAST column of V
    x_hom = V(:, end);
    
    % De-homogenize (Convert from [x,y,z,w] to [x,y,z])
    x_world = x_hom(1:3) / x_hom(4);

    % Verify it is in front of at least the first camera
    X_cam_check = T_world_camera * [x_world; 1];
    if X_cam_check(3) <= 0
        x_world = [];  % Handling invalid point (options: NaN or empty)
    end
end



%%%%%%%%%%%
% Utilities
%%%%%%%%%%%

% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function v=t2v(A)
	v(1:2, 1) = A(1:2,3);
	v(3, 1) = atan2(A(2,1), A(1,1));
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