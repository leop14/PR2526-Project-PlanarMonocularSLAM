function [rmse_translation, rmse_rotation] = evaluate_traj(XR_est, traj_data)
    % Relative pose error: for each pair of consecutive poses, compare the
    % estimated relative motion with the ground truth one. Returns the RMSE
    % of the translation and rotation errors over all pairs.

    num_poses = size(XR_est, 3);
    sum_trans_error_sq = 0;
    sum_rot_error_sq = 0;

    for i = 1:(num_poses - 1)
        % Compute Estimated Relative Motion
        T_0 = XR_est(:, :, i);
        T_1 = XR_est(:, :, i+1);
        rel_T = inv(T_0) * T_1;
        
        % Compute Ground Truth Relative Motion
        GT_0 = v2t(traj_data.gt(i, :));
        GT_1 = v2t(traj_data.gt(i+1, :));
        rel_GT = inv(GT_0) * GT_1;
        
        % Compute SE(2) Error
        error_T = inv(rel_T) * rel_GT;
        
        % Extract Errors
        rot_err = atan2(error_T(2, 1), error_T(1, 1));
        trans_err_sq = error_T(1, 4)^2 + error_T(2, 4)^2;
        
        sum_rot_error_sq = sum_rot_error_sq + rot_err^2;
        sum_trans_error_sq = sum_trans_error_sq + trans_err_sq;
    end

    rmse_translation = sqrt(sum_trans_error_sq / (num_poses - 1));
    rmse_rotation = sqrt(sum_rot_error_sq / (num_poses - 1));

    fprintf('Trajectory Translation RMSE: %.6f meters\n', rmse_translation);
    fprintf('Trajectory Rotation RMSE: %.6f radians\n', rmse_rotation);
end

