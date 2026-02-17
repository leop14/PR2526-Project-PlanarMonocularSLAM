close all 
clear
clc 


cam_data = read_camera_data("data/camera.dat");


traj_data = read_traj_data("data/trajectory.dat");


%draw_traj(traj_data);
%disp("Map displayed. Press any key to close and exit");
%pause;

meas_data = read_meas_data("data");
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