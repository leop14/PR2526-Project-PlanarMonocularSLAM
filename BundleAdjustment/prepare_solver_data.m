function [XR_guess, XL_guess, Zr, pose_associations, Zp, projection_associations, landmark_ids, num_poses, num_landmarks] = prepare_solver_data(world_map, traj_data, meas_db)

    % Building XL_guess and the ID Lookup Table 
    % by extracting the triangulated landmarks
    landmark_ids = cell2mat(keys(world_map));
    num_landmarks = length(landmark_ids);
    XL_guess = zeros(3, num_landmarks);

    % Dictionary to map Actual Dataset ID -> Dense Matrix Column Index (1 to N)
    id_to_col = containers.Map('KeyType', 'int32', 'ValueType', 'int32');

    for col = 1:num_landmarks
        l_id = landmark_ids(col);
        XL_guess(:, col) = world_map(l_id); % Storing [x;y;z] into the column
        id_to_col(l_id) = col;              
    end

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Building XR_guess and Zr (Poses and Odometry Constraints) 
    num_poses = length(traj_data.id);
    XR_guess = zeros(4, 4, num_poses);

    num_pose_measurements = num_poses - 1;
    Zr = zeros(3, num_pose_measurements); % SE(2) odometry constraints [dx; dy; dtheta]
    pose_associations = zeros(2, num_pose_measurements);

    for i = 1:num_poses
        % Using v2t to convert [x, y, theta] odometry to 4x4 initial guess
        XR_guess(:,:,i) = v2t(traj_data.odom(i, :)); 
        
        % Building relative odometry constraints (Zr)
        if i < num_poses
            pose_i = traj_data.odom(i, :);
            pose_j = traj_data.odom(i+1, :);
            
            % Computing relative motion between consecutive odometry readings
            T_i = v2t(pose_i);
            T_j = v2t(pose_j);
            T_rel = inv(T_i) * T_j;
            
            dx = T_rel(1,4);
            dy = T_rel(2,4);
            dtheta = atan2(T_rel(2,1), T_rel(1,1));
            
            Zr(:, i) = [dx; dy; dtheta];
            pose_associations(:, i) = [i; i+1];
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Building Zp and projection_associations 
    % Using dynamic arrays since the total number of valid observations varies
    Zp_list = [];
    proj_assoc_list = [];

    for i = 1:num_landmarks
        l_id = landmark_ids(i);
        col_idx = id_to_col(l_id);
        obs_struct = meas_db(l_id);
        
        for k = 1:obs_struct.count
            obs = obs_struct.observations{k};

            pose_idx = obs.seq_num + 1;  % Octave is 1-indexed
            
            Zp_list(:, end+1) = obs.uv';
            proj_assoc_list(:, end+1) = [pose_idx; col_idx];
        end
    end

    Zp = Zp_list;
    projection_associations = proj_assoc_list;
end