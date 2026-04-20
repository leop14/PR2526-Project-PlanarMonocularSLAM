function v_idx = landmarkMatrixIndex(landmark_index, num_poses, num_landmarks)
    global pose_dim;
    global landmark_dim;

    if (landmark_index > num_landmarks)
        v_idx = -1;
        return;
    endif;
    v_idx = 1 + num_poses * pose_dim + (landmark_index - 1) * landmark_dim;
endfunction
