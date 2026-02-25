close all 
clear
clc 


cam_data = read_camera_data("data/camera.dat");


traj_data = read_traj_data("data/trajectory.dat");


%draw_traj(traj_data);
%disp("Map displayed. Press any key to close and exit");
%pause;

meas_data_db = new_read_meas_data("data");

% Testing correctness: Verify a specific point (e.g. Point #6)
if isKey(meas_data_db, 6)
    pt = meas_data_db(6);
    fprintf('\nChecking Point ID #6\n');
    fprintf('\tObserved in %d frames.\n', pt.count);
    
    obs = pt.observations{1};
    fprintf('\tFirst obs: Frame %d at pixels [%.2f, %.2f]\n', ...
            obs.seq_num, obs.uv(1), obs.uv(2));
    fprintf('\tRobot Odom at that time: [%.4f, %.4f, %.4f]\n', obs.odom_pose);

    obs = pt.observations{pt.count};
    fprintf('\tLast obs: Frame %d at pixels [%.2f, %.2f]\n', ...
            obs.seq_num, obs.uv(1), obs.uv(2));
    fprintf('\tRobot Odom at that time: [%.4f, %.4f, %.4f]\n', obs.odom_pose);

end


world_gt_map = read_world_data("data/world.dat");


disp("Triangulating Points - method 1");
map_estimate = triangulate1(meas_data_db, cam_data.T, cam_data.K);


%%%%%%%%%%%%%
% EVALUATION
%%%%%%%%%%%%%
disp("Evaluating Map Quality");


% We want to compute the whole RMSE


squared_error_sum = 0;
count_evaluated = 0;

estimated_ids = cell2mat(keys(map_estimate));
gt_points = [];
est_points = [];

for i = 1:length(estimated_ids)
    id = estimated_ids(i);
    
    % Only evaluate if we have ground truth for this ID
    if isKey(world_gt_map, id)
        p_est = map_estimate(id);
        p_gt = world_gt_map(id);

        if isempty(p_est)
            continue;
        end
        
        % Accumulate error
        diff = p_est - p_gt;
        squared_error_sum = squared_error_sum + sum(diff.^2);
        count_evaluated = count_evaluated + 1;
        
        % Store for plotting
        gt_points(:, end+1) = p_gt;
        est_points(:, end+1) = p_est;
    end
end

if count_evaluated > 0
    rmse = sqrt(squared_error_sum / count_evaluated);
    fprintf('\n');
    fprintf('Map RMSE: %.4f meters\n', rmse);
    fprintf('Evaluated %d points.\n', count_evaluated);
else
    warning('No overlapping points found between Estimate and GT!');
end

%%%%%%%%%%
% Visualization 
%%%%%%%%%%

draw_3D_points(est_points, gt_points, rmse);



%%%%%%%%%%%%%
% EVALUATION - Triang 2
%%%%%%%%%%%%%
disp("Triangulating Points - method 2");
map_estimate = triangulate2(meas_data_db, cam_data.T, cam_data.K);

disp("Evaluating Map Quality");


% We want to compute the whole RMSE


squared_error_sum = 0;
count_evaluated = 0;

estimated_ids = cell2mat(keys(map_estimate));
gt_points = [];
est_points = [];

for i = 1:length(estimated_ids)
    id = estimated_ids(i);
    
    % Only evaluate if we have ground truth for this ID
    if isKey(world_gt_map, id)
        p_est = map_estimate(id);
        p_gt = world_gt_map(id);

        if isempty(p_est)
            continue;
        end
        
        % Accumulate error
        diff = p_est - p_gt;
        squared_error_sum = squared_error_sum + sum(diff.^2);
        count_evaluated = count_evaluated + 1;
        
        % Store for plotting
        gt_points(:, end+1) = p_gt;
        est_points(:, end+1) = p_est;
    end
end

if count_evaluated > 0
    rmse = sqrt(squared_error_sum / count_evaluated);
    fprintf('\n');
    fprintf('Map RMSE: %.4f meters\n', rmse);
    fprintf('Evaluated %d points.\n', count_evaluated);
else
    warning('No overlapping points found between Estimate and GT!');
end

%%%%%%%%%%
% Visualization 
%%%%%%%%%%

draw_3D_points(est_points, gt_points, rmse);