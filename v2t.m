% From a planar pose [x, y, theta] to a 4x4 homogeneous transform
% (rotation around z, translation in the xy plane)
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
