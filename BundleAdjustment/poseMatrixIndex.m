function v_idx = poseMatrixIndex(pose_index, num_poses, num_landmarks)
    global pose_dim;
    global landmark_dim;

    if (pose_index > num_poses)
        v_idx = -1;
        return;
    endif;
    v_idx = 1 + (pose_index - 1) * pose_dim;
endfunction
