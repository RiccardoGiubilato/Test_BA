function err = reproj(obs, point, Cam, K)
% @input obs,   2x1 vector of u, v coordinates in the image
% @input point, 3x1 vector of 3D point coordinates in world reference
% @input P,     _x4 camera matrix K*[R|t]

err = sqrt(sum((obs - proj(point, Cam, K)).^2));

end