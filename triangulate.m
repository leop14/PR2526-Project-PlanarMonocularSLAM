
function trangulation = triangulate(meas_db, T_camera_robot, K)
    % Initializing the output map (ID -> [x; y; z])
    triangulation = containers.Map('KeyType', 'int32', 'ValueType', 'any');

    % Getting all keys (i.e. Landmark IDs) from the database
    landmark_ids = cell2mat(keys(meas_db));

    fprintf('Triangulating %d points.\n', length(landmark_ids));

    num_points = length(landmark_ids);
    for 1:num_points     % Iteration over all the points
        point_id = landmark_ids(i);
        point_struct = meas_db(id);

        if point_struct.count < 2
            continue;
        end
        
        % Perform triangulation on the current point
        x_world = single_point_triangulation(point_struct, T_camera_robot, K);

        % Store the result (Euclidean 3D point)
        triangulation(id) = X_world;
    end
    
    fprintf('Triangulation complete. %d points have been mapped.\n', length(triangulation));
end



function single_triang = single_point_triangulation(point_struct, T_camera_robot, K)
    n_frames = point_struct.count;
    for frame = 1:n_frames
        frame_infos = point_struct.observations{frame};
        frame_odom_pose = frame_infos.odom_pose;
        T_robot_world = v2t(frame_odom_pose);

        T_camera_world = T_robot_world * T_camera_robot;
        T_world_camera = inv(T_camera_world);

        R = T_world_camera(1:3, 1:3);
        t = T_world_camera(1:3, 4);
        T = (R, t);
        P = K * T;
    end
end