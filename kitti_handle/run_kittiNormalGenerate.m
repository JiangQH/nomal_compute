function run_kittiNormalGenerate(base_dir, calib_dir)
% base_dir: the dir contains sequences. .. img & velodyne
% calib_dir: the dir contains the calib file
clear; clc;
addpath('./matlab/');
addpath('./toolbox/');
addpath('./toolbox/tv_denoise');
if nargin<1
  base_dir  = 'H:\Jiang\Dataset\kitti_raw\data\2011_09_26\2011_09_26_drive_0005_sync';
end
if nargin<2
  calib_dir = 'H:\Jiang\Dataset\kitti_raw\calib\2011_09_26';
end
cam       = 2; % 0-based index
frame     = 134; % 0-based index
% load calibration
calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));

% compute projection matrix velodyne->image plane. velodyne->camera space
R_cam_to_rect = eye(4);
R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
P_velo_to_cam = R_cam_to_rect*Tr_velo_to_cam;
P_velo_to_img = calib.P_rect{cam+1}*P_velo_to_cam;
img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
[Iy, Ix, ~] = size(img);
fig = figure('Position',[20 100 size(img,2) size(img,1)]); axes('Position',[0 0 1 1]);
imshow(img); hold on;

% load velodyne points
fid = fopen(sprintf('%s/velodyne_points/data/%010d.bin',base_dir,frame),'rb');
velo = fread(fid,[4 inf],'single')';
fclose(fid);

% project to image plane (exclude luminance). remove all the points that
% behind the plane and out of image range
[velo_img, plist] = project(velo(:,1:3),P_velo_to_img);
velo(plist, :) = [];
velo_img(plist, :) = [];
plist = velo_img(:, 1) > 0.5 & velo_img(:, 2) > 0.5 & velo_img(:, 1) < (Ix + 0.5) & velo_img(:, 2) < (Iy + 0.5);
velo = velo(plist,:);
velo_img = velo_img(plist, :);
velo_index = round(velo_img);


interpted_velo = [velo_index(:,2), velo_index(:,1), velo(:,1:3)];
% interpted_velo = interpvelo(interpted_velo, 10);
% % % % 
% interpted_velo = [interpted_velo(:,2), interpted_velo(:,1), interpted_velo(:,3:5)];
% interpted_velo = interpvelo(interpted_velo, 10);
% interpted_velo = [interpted_velo(:,2), interpted_velo(:,1), interpted_velo(:,3:5)];
% generate a 3d matrix containing the 3d points
velo_in_cam = zeros(Iy, Ix, 3);
velo_cam = project(interpted_velo(:,3:5), P_velo_to_cam);
for i = 1 : size(interpted_velo, 1)
    velo_in_cam(round(interpted_velo(i,1)), round(interpted_velo(i,2)), :) = velo_cam(i,:);
end
filled = fillhole(velo_in_cam);
% filled = velo_in_cam;
K = 0.125 * ones(3);
s1 = conv2(filled(:,:,1), K, 'same');
s2 = conv2(filled(:,:,2), K, 'same');
s3 = conv2(filled(:,:,3), K, 'same');
index1 = (s1 == 0);
index2 = (s2 == 0);
index3 = (s3 == 0);

% resize to accelerate
% sampleFactor = 1;
% bilateralSigmaSpatial = 24;
% bilateralSigmaIntensity = 0.2;
% filled = imresize(filled, 1.0 / sampleFactor);

% get the Xd, Yd, Zd
Xd = s1;
Yd = s2;
Zd = s3;
% s = cat(3,s1,s2,s3);
% fid = fopen('1242-375.txt','wb');
% fwrite(fid, s, 'float');
% fclose(fid);

% do fill job?
% im = imresize(img, 1.0 / sampleFactor);
% Xd = fill_depth_colorization(double(im)./255.0, Xd, 0.5);
% Yd = fill_depth_colorization(double(im)./255.0, Yd, 0.5);
% Zd = fill_depth_colorization(double(im)./255.0, Zd, 0.5);
% Xd = imresize(Xd, [Iy, Ix],'method','bilinear');
% Yd = imresize(Yd, [Iy, Ix],'method','bilinear');
% Zd = imresize(Zd, [Iy, Ix],'method','bilinear');
% Xd(index) = 0;
% Yd(index) = 0;
% Zd(index) = 0;

% compute the normal. 2 ways here.
% first
% [Nxd, Nyd, Nzd] = surfnorm(Xd, Yd, Zd);
% second
projectionSize = size(Xd);
[imgPlanes, imgNormals, normalConf, NCompute] = ...
              		compute_local_planes(Xd, Yd, Zd, projectionSize);
% smooth the normal. two way: 1. tvNormal. 2. bfilter2
% 1¡¢bfilter2
% NxU = (Nxd + 1) / 2; NyU = (Nyd + 1) / 2; NzU = (Nzd + 1) / 2;
% NxU = bfilter2(NxU, bilateralSigmaSpatial*3, [bilateralSigmaSpatial,bilateralSigmaIntensity]);
% NyU = bfilter2(NyU, bilateralSigmaSpatial*3, [bilateralSigmaSpatial,bilateralSigmaIntensity]);
% NzU = bfilter2(NzU, bilateralSigmaSpatial*3, [bilateralSigmaSpatial,bilateralSigmaIntensity]);
% NxU = 2*NxU - 1;
% NyU = 2*NyU - 1;
% NzU = 2*NzU - 1;

% 2¡¢tv-denoise
Ndash  = tvNormal(NCompute,1);
Nx = Ndash(:,:,1);
Ny = Ndash(:,:,2);
Nz = Ndash(:,:,3);

% resize back to the original size
% Nx = imresize(Nx, [Iy, Ix],'method','bilinear');
% Ny = imresize(Ny, [Iy, Ix],'method','bilinear');
% Nz = imresize(Nz, [Iy, Ix],'method','bilinear');
N = Nx.^2 + Ny.^2 + Nz.^2;
N = N.^0.5;
Nx = Nx ./ N; Ny = Ny ./ N; Nz = Nz ./ N;
Nx(index1) = 0;
Ny(index2) = 0;
Nz(index3) = 0;
n = cat(3, Nx, Ny, Nz);
figure(3);
imshow(n);

% for i = 1 : size(interpted_velo,1)
%     col_idx = round(64*5/interpted_velo(i,3));
%     plot(interpted_velo(i,2), interpted_velo(i,1),'o','LineWidth', 1,'MarkerSize',1,'Color',cols(col_idx,:));
% end




end

