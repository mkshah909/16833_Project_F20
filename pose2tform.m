function tform = pose2tform(pose)
% convert 2D pose to transformation matrix
% [input] pose: [x, y, theta], theta in radians
% [output] tform: 3D rigid body transform
tform = [cos(pose(3)), -sin(pose(3)), 0, pose(1);
         sin(pose(3)),  cos(pose(3)), 0, pose(2);
                    0,             0, 1,       0;
                    0,             0, 0,       1];
tform = affine3d(tform');

end