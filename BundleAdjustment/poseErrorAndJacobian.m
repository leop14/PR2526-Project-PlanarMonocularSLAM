# error and jacobian of a relative SE(2) odometry constraint between two poses
function [e, Ji, Jj] = poseErrorAndJacobian(Xi, Xj, Z)
    # predicted relative transform
    H = inv(Xi) * Xj;

    # generators of the right-multiplicative SE(2) perturbation v2t([dx,dy,dtheta])
    G1 = [0 0 0 1; 0 0 0 0; 0 0 0 0; 0 0 0 0]; # d/dx
    G2 = [0 0 0 0; 0 0 0 1; 0 0 0 0; 0 0 0 0]; # d/dy
    G3 = [0 -1 0 0; 1 0 0 0; 0 0 0 0; 0 0 0 0]; # d/dtheta

    e = flattenBlock(H) - flattenBlock(Z);

    Jj = zeros(6, 3);
    Jj(:, 1) = flattenBlock(H * G1);
    Jj(:, 2) = flattenBlock(H * G2);
    Jj(:, 3) = flattenBlock(H * G3);

    Ji = zeros(6, 3);
    Ji(:, 1) = flattenBlock(-G1 * H);
    Ji(:, 2) = flattenBlock(-G2 * H);
    Ji(:, 3) = flattenBlock(-G3 * H);
endfunction

# extracts the planar rotation (2x2) and translation (2x1) part of a 4x4 SE(2) matrix
function v = flattenBlock(M)
    v = [M(1,1); M(2,1); M(1,2); M(2,2); M(1,4); M(2,4)];
endfunction
