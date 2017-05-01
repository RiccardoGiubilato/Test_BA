function obs = proj(point, Cam, K)
% @input point, 3x1 vector of 3D point coordinates in world reference
% @input Cam,   4x4 transformation matrix of the camera
% @input K,     3x3 intrinsic camera matrix

if size(point,1) ~= 3
    error('point must be 3x1 vector')
end

% transform to local coordinates
point = [Cam; 0 0 0 1]^-1 * [point; 1];
point = point(1:3);

% normalize
point = point/point(3);

% transform to image coordinates
obs = K*point;

% keep u and v
obs = obs(1:2)';

end

