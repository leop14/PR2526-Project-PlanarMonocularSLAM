close all 
clear
clc 


cam_data = read_camera_data("data/camera.dat");


traj_data = read_traj_data("data/trajectory.dat");


%draw_traj(traj_data);
%disp("Map displayed. Press any key to close and exit");
%pause;

meas_data_db = new_read_meas_data("data");
% Testing correctness

% for frame_idx = 1:3
%     frame = meas_data{frame_idx};
%     fprintf('Sequence num:\t %d \n', frame.id);
%     fprintf('Odometry:\t [%.6f, %.6f, %.6f]\n', frame.odom_pose);
%     fprintf('GroundTruth:\t [%.6f, %.6f, %.6f]\n', frame.gt_pose);
%     if ~isempty(frame.point_ids)
%         fprintf('ID=%d | u=%.3f | v=%.3f \n\n', ...
%             frame.point_ids(1), frame.observations(1,1), frame.observations(1,2));
%     else
%         warning('Frame %d has no features\n', frame_idx);
%     end
% end

% Verify a specific point (e.g. Point #6)
if isKey(meas_data_db, 6)
    pt = meas_data_db(6);
    fprintf('\nChecking Point ID #6\n');
    fprintf('\tObserved in %d frames.\n', pt.count);
    
    first_obs = pt.observations{1};
    fprintf('\tFirst obs: Frame %d at pixels [%.2f, %.2f]\n', ...
            first_obs.seq_num, first_obs.uv(1), first_obs.uv(2));
    fprintf('\tRobot Odom at that time: [%.4f, %.4f, %.4f]\n', first_obs.odom_pose);

    first_obs = pt.observations{pt.count};
    fprintf('\tLast obs: Frame %d at pixels [%.2f, %.2f]\n', ...
            first_obs.seq_num, first_obs.uv(1), first_obs.uv(2));
    fprintf('\tRobot Odom at that time: [%.4f, %.4f, %.4f]\n', first_obs.odom_pose);

end