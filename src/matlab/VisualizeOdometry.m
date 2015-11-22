function [trajectory, map] = VisualizeOdometry(trajectory_file, map_file)
% VisualizeOdometry.m

% Load the trajectory.
trajectory = LoadTrajectory(trajectory_file);

% Load the map file.
map = LoadMap(map_file);

% Initialize a new figure.
figure(1); clf; grid on; box on;

% Draw the camera's trajectory.
axis_scale = 1;
DrawPoses(trajectory, axis_scale);

% Draw all 3D landmark positions.
DrawMap(map);

end

%% Draw the trajectory as a sequence of rotated coordinate frames.
function DrawPoses(trajectory, scale)

% Get translations and rotations. Rotations are inverted, hence the - sign.
%
t = cat(1, trajectory(:).translation);
r = -cat(1, trajectory(:).rotation);

% Convert all axis angle rotations to quaternions.
q = AxisAngle2Quat(r);

% Rotate the standard bases for quiver plots.
ax = RotateVector(q, scale*[1 0 0]);
ay = RotateVector(q, scale*[0 1 0]);
az = RotateVector(q, scale*[0 0 1]);

% Need to permute our coordinates. camera's +z is forward, +x is right, and
% +y is down. Matlab visualization's +z is up, +x is forward, and +y is
% left.
figure(1); hold on;
fx = quiver3(t(:,3), -t(:,1), -t(:,2), ax(:,3), -ax(:,1), -ax(:,2), 'r');
fy = quiver3(t(:,3), -t(:,1), -t(:,2), ay(:,3), -ay(:,1), -ay(:,2), 'g');
fz = quiver3(t(:,3), -t(:,1), -t(:,2), az(:,3), -az(:,1), -az(:,2), 'b');
plot3(t(:,3), -t(:,1), -t(:,2), '--k');
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
q(isnan(q)) = 0;
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

% Need to permute our coordinates. camera's +z is forward, +x is right, and
% +y is down. Matlab visualization's +z is up, +x is forward, and +y is
% left.
p = ([0 0 1; -1 0 0; 0 -1 0] * p')';
showPointCloud(p);
colormap(jet);

% Get the point furthest from the origin so we can set our axis.
d = sqrt(sum(abs(p).^2, 2));
[~,index] = max(d);
p_max = abs(p(index,:));
axis([-p_max(1) p_max(1) -p_max(2) p_max(2) -p_max(3) p_max(3)]);
end
