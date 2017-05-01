clearvars
close all

nPoints = 100;
nCamera = 8;
depth   = 1000; %[mm]
fSx     = 1000; %[px/mm*mm]
fSy     = 1000;
cx      = 1000; %[px]
cy      = 500;  %[px]
FOV     = [80, 60]; %hor, vert [�]

depth_noise = depth/20;
angle_noise = 3*pi/180;
trans_noise = depth/20;

use_finite_diff = 0; % 1 for finite differences, 0 for levenberg-marquardt

%% Pose and Point Cloud Generation
Points  = depth*[-0.5+rand(nPoints,2) ones(nPoints,1)];
Points(:,1) = nCamera/5*Points(:,1);
Camera = zeros(4,4,nCamera);
dist_factor = 1*depth; % adjust this parameter to set a correct spacing
                       % between the cameras
for i=1:nCamera
    t = dist_factor*[(-0.5 + (i-1)/nCamera);0;0];
    Camera(:,:,i) = [eye(3) t; 0 0 0 1];
end
 
%% Observation Generation (u,v) coordinates of "image features"
Obs = cell(1,nCamera); %[u,v,pointdIdx]
%Err = cell(1,nCamera);
K = [fSx 0 cx; 0 fSy cy; 0 0 1];
for n=1:nCamera
    for i=1:nPoints
         if pointIsVisible( Points(i,:)', Camera(1:3,:,n), FOV )
           Obs{n}(end+1,:) = [proj( Points(i,:)', Camera(1:3,:,n), K ), i];
           %Err{n}(end+1) = reproj (Obs{n}(end,1:2), Points(i,:)', Camera(1:3,:,n), K);
         end
    end
end

%% Adding noise to Points e Camera
Points = Points + depth_noise*randn(size(Points,1), size(Points, 2));
max_angular_error = angle_noise;
for i=1:nCamera
    rotMatrix = eul2rotm(randn(1,3)*max_angular_error);
    Camera(1:3,1:3,i) = rotMatrix * Camera(1:3,1:3,i);
    Camera(1:3,4,i) = Camera(1:3,4,i) + randn(3,1) * trans_noise;
end

Err = cell(1,nCamera);
for n=1:nCamera
    for i=1:size(Obs{n},1)
       Err{n}(i) = reproj (Obs{n}(i,1:2), Points(Obs{n}(i,3),:)', Camera(1:3,:,n), K);
    end
end

%% Plot before Bundle Adjustment
figure
scatter3(Points(:,1),Points(:,2),Points(:,3),'c')
hold on
axis equal
x=zeros(1,nCamera); for i=1:nCamera, x(i) = Camera(1,4,i); end
y=zeros(1,nCamera); for i=1:nCamera, y(i) = Camera(2,4,i); end
z=zeros(1,nCamera); for i=1:nCamera, z(i) = Camera(3,4,i); end
v=zeros(3,nCamera); for i=1:nCamera, v(:,i) = Camera(1:3,1:3,i)*[0;0;1]; end

quiver3(x,y,z, ...
        depth/5*v(1,:),depth/5*v(2,:),depth/5*v(3,:))
title('Points and Camera Views - Before BA')

figure
for i=1:nCamera
   subplot(4,nCamera/4,i)
   scatter(Obs{i}(:,1),Obs{i}(:,2),'c')
   projectons = zeros(nPoints,2);
   for j=1:nPoints
       if any(Obs{i}(:,3)==j)
        projectons(j,:) = proj(Points(j,:)', Camera(1:3,:,i), K);
       end
   end
   hold on 
   scatter(projectons(:,1),projectons(:,2),'.')
end
title('Camera image frames. Measures and reprojections. Before BA')

%% Bundle Adjustment - reprojection
[Points, Camera] = bundleAdj(Points, Camera, Obs, K, use_finite_diff);

ErrBA = cell(1,nCamera);
for n=1:nCamera
    for i=1:size(Obs{n},1)
       ErrBA{n}(i) = reproj (Obs{n}(i,1:2), Points(Obs{n}(i,3),:)', Camera(1:3,:,n), K);
    end
end

%% Plot after Bundle Adjustment
fprintf('Mean reprojection error before BA: %02d \n', mean(cellfun(@mean,Err)))
fprintf('Mean reprojection error after BA:  %02d   \n', mean(cellfun(@mean,ErrBA)))

figure
scatter3(Points(:,1),Points(:,2),Points(:,3),'c')
hold on
axis equal
x=zeros(1,nCamera); for i=1:nCamera, x(i) = Camera(1,4,i); end
y=zeros(1,nCamera); for i=1:nCamera, y(i) = Camera(2,4,i); end
z=zeros(1,nCamera); for i=1:nCamera, z(i) = Camera(3,4,i); end
v=zeros(3,nCamera); for i=1:nCamera, v(:,i) = Camera(1:3,1:3,i)*[0;0;1]; end

quiver3(x,y,z, ...
        depth/5*v(1,:),depth/5*v(2,:),depth/5*v(3,:))
title('Points and Camera Views - After BA')

figure
for i=1:nCamera
   subplot(4,nCamera/4,i)
   scatter(Obs{i}(:,1),Obs{i}(:,2),'c')
   projections = zeros(nPoints,2);
   for j=1:nPoints
       if any(Obs{i}(:,3)==j)
        projections(j,:) = proj(Points(j,:)', Camera(1:3,:,i), K);
       end
   end
   hold on 
   scatter(projections(:,1),projections(:,2),'.')
end
title('Camera image frames. Measures and reprojections. After BA')
