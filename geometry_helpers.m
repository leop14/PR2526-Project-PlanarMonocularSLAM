1;
%Try to put "1;" here?

% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
% function v=t2v(A)
% 	v(1:2, 1) = A(1:2,3);
% 	v(3, 1) = atan2(A(2,1), A(1,1));
% end

% computes the homogeneous transform matrix A of the pose vector v
% A: 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  3D pose vector
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