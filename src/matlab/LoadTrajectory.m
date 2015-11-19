function [trajectory] = LoadTrajectory(trajectory_file)
% Load a trajectory file, unpacking its poses into an array with elements:
%
% view_index  - the index of the view that the pose was captured from.
% translation - the translation of the camera.
% rotation    - the axis angle rotation of the camera.

raw_trajectory = csvread(trajectory_file);

% Iterate over all views, storing their elements as a camera pose object.
n_poses = size(raw_trajectory, 1);
trajectory = repmat(struct('view_index', 0, 'translation', 0, 'rotation', 0), n_poses, 1);
for ii = 1 : n_poses
    pose.view_index = raw_trajectory(ii, 1);
    pose.translation = raw_trajectory(ii, 2:4);
    pose.rotation = raw_trajectory(ii, 5:7);
    trajectory(ii) = pose;
end

end

