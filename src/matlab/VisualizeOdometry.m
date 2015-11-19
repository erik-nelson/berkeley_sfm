function VisualizeOdometry(trajectory_file, map_file)
% VisualizeOdometry.m

% Load the trajectory.
trajectory = LoadTrajectory(trajectory_file);

% Load the map file.
map = LoadMap(map_file);

% Initialize a new figure.
figure(1); clf;

% Draw the camera's trajectory.
axis_scale = 5;
DrawPoses(trajectory, axis_scale);

% Draw all 3D landmark positions.
DrawMap(map);

end

%% Draw the trajectory as a sequence of rotated coordinate frames.
function DrawPoses(trajectory, scale)

% Get translations and rotations.
t = cat(1, trajectory(:).translation);
r = cat(1, trajectory(:).rotation);

% Convert all axis angle rotations to quaternions.
q = AxisAngle2Quat(r);

% Rotate the standard bases for quiver plots.
ax = RotateVector(q, scale*[1 0 0]);
ay = RotateVector(q, scale*[0 1 0]);
az = RotateVector(q, scale*[0 0 1]);

% Flip z and x, since the camera's forward direction is +z.
figure(1); hold on;
fx = quiver3(t(:,3), t(:,1), t(:,2), ax(:,3), ax(:,1), ax(:,2), 'r');
fy = quiver3(t(:,3), t(:,1), t(:,2), ay(:,3), ay(:,1), ay(:,2), 'g');
fz = quiver3(t(:,3), t(:,1), t(:,2), az(:,3), az(:,1), az(:,2), 'b');
axis equal;
set(fx, 'showarrowhead', 'off', 'autoscale', 'off');
set(fy, 'showarrowhead', 'off', 'autoscale', 'off');
set(fz, 'showarrowhead', 'off', 'autoscale', 'off');

end

%% Convert a Nx3 set of axis-angle vectors into an Nx4 set of quaternions.
function q = AxisAngle2Quat(v)
q = zeros(size(v,1),4);
theta = sqrt(sum(v.^2, 2));
q(:,1) = cos(theta/2);
q(:,2:4) = bsxfun(@times, v, sin(theta/2)./theta);
end

%% Rotate an Nx3 set of vectors by an Mx4 set of quaterions.
function v = RotateVector(q, v)
q_out = quatmultiply(quatmultiply(q,[zeros(size(v,1),1),v]),bsxfun(@times,[1,-1,-1,-1],q));
v = q_out(:,2:4);
end

%% Draw the set of landmarks as a pointcloud.
function DrawMap(map)
% Get the positions of all landmarks.
p = cat(1, map(:).position);

% Flip z and x, since the camera's forward direction is +z.
showPointCloud(p(:,[3,1,2]));
end