% computes the homogeneous transform matrix A of the pose vector v
% A: 4x4 homogeneous transformation matrix (planar motion, z=0)
% v: [x,y,theta]  2D pose vector
function T = v2t(pose)
    tx = pose(1);
    ty = pose(2);
    theta = pose(3);

    c = cos(theta);
    s = sin(theta);

    T = [c, -s, 0, tx;
         s,  c, 0, ty;
         0,  0, 1, 0;
         0,  0, 0, 1];
end
