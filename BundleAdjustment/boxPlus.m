# implementation of the boxplus for SE(2) planar poses and 3D landmarks
# applies a perturbation to a set of landmarks and robot poses
# input:
#   XR: the robot poses (4x4xnum_poses: array of homogeneous matrices)
#   XL: the landmark pose (3xnum_landmarks matrix of landmarks)
#   num_poses: number of poses in XR
#   num_landmarks: number of landmarks in XL
#   dx: the perturbation vector of appropriate dimensions (poses first, then landmarks)
# output:
#   XR: the robot poses obtained by applying the perturbation
#   XL: the landmarks obtained by applying the perturbation

function [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx)
    global pose_dim;      
    global landmark_dim;  
    
    # Update Poses
    for(pose_index = 1:num_poses)
        pose_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks);
        dxr = dx(pose_matrix_index : pose_matrix_index+pose_dim-1);
        
        # Constructing the SE(2) perturbation matrix (4x4)
        dX = v2t(dxr);
            
        # Apply the perturbation to the pose
        XR(:,:,pose_index) = dX * XR(:,:,pose_index);
    endfor;
    
    # Update Landmarks
    for(landmark_index = 1:num_landmarks)
        landmark_matrix_index = landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);
        dxl = dx(landmark_matrix_index : landmark_matrix_index+landmark_dim-1, :);
        
        # 3D landmarks are in Euclidean space
        XL(:,landmark_index) += dxl;
    endfor;
endfunction;



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