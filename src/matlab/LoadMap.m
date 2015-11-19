function [map] = LoadMap(map_file)
% Load a map file, unpacking its landmarks into an array with elements:
%
% position     - the 3D position of the landmark.
% view_indices - indices of views that observed the landmark.

fid = fopen(map_file);
raw_map = textscan(fid, '', -1, 'delimiter', ',',...
               'whitespace', sprintf(' \b\t'),...
               'emptyvalue', NaN, 'collectoutput', true);
raw_map = raw_map{1};
fclose(fid);

% Iterate over all landmarks, storing their information in an array of
% structs.
n_points = size(raw_map, 1);
map = repmat(struct('position', 0, 'view_indices', 0), n_points, 1);
for ii = 1 : n_points
    point.position = raw_map(ii,1:3);
    point.view_indices = raw_map(ii,4:end);
    map(ii) = point;
end

end

