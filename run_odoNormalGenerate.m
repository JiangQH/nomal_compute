function run_odoNormalGenerate(base_dir)
% base_dir: the dir contains sequences. .. img & velodyne
% calib_dir: the dir contains the calib file
addpath('./matlab/');
addpath('./toolbox/');
addpath('./toolbox/tv_denoise');
if nargin<1
  base_dir  = '/run/user/1000/gvfs/smb-share:server=10.78.92.49,share=data/Kitti/odometry';
end
calib_dir = [base_dir, '/', 'calib'];
velodyne_dir = [base_dir, '/', 'velodyne'];
rgb_dir = [base_dir, '/', 'gray'];
% save dir
normal_save_dir = [base_dir, '/handled/', 'normal'];
normal_map_save_dir = [base_dir, '/handled/', 'normalmap'];
img_save_dir = [base_dir, '/handled/', 'rgb'];
% if ~exist(normal_save_dir, 'dir')
%     mkdir(normal_save_dir);
% end
% if ~exist(normal_map_save_dir, 'dir')
%     mkdir(normal_map_save_dir);
% end
% if ~exist(img_save_dir, 'dir')
%     mkdir(img_save_dir);
% end
sequences = dir(calib_dir);
for seq = 3 : length(sequences)
    calib_file = [calib_dir, '/', sequences(seq).name, '/calib.txt'];
    rgb_seq_dir = [rgb_dir, '/', sequences(seq).name, '/image_0'];
    velo_seq_dir = [velodyne_dir, '/', sequences(seq).name, '/velodyne'];
    % load the calib
    [P, Tr] = load_calib(calib_file);
    P_velo_to_cam = [Tr; 0,0,0,1]; 
    P_velo_to_img = P * P_velo_to_cam;
    % load every img and velo points
    imgs = dir(rgb_seq_dir);
    for id = 3 : 10 : length(imgs)
        img_file = [rgb_seq_dir, '/', imgs(id).name];
        [~, name, ~] = fileparts(imgs(id).name);
        velo_file = [velo_seq_dir, '/', name, '.bin'];
        if ~exist(velo_file, 'file')
            continue;
        end
        % load the img and velo
        img = imread(img_file);
        [Iy, Ix, ~] = size(img);
        fid = fopen(velo_file, 'rb');
        velo = fread(fid, [4 inf], 'single')';
        fclose(fid);
        % project to image plane remove points
        [velo_img, plist] = project(velo(:,1:3),P_velo_to_img);
        velo(plist, :) = [];
        velo_img(plist, :) = [];
        plist = velo_img(:, 1) > 0.5 & velo_img(:, 2) > 0.5 & velo_img(:, 1) < (Ix + 0.5) & velo_img(:, 2) < (Iy + 0.5);
        velo = velo(plist,:);
        velo_img = velo_img(plist, :);
        velo_index = round(velo_img);
        idx = round(64*5./velo(:,1));
        
        interpted_velo = [idx, velo_index(:,2), velo_index(:,1), velo(:,1:3)];
        velo_in_cam = zeros(Iy, Ix, 4);
        velo_cam = project(interpted_velo(:,4:6), P_velo_to_cam);
        for i = 1 : size(interpted_velo, 1)
            velo_in_cam(interpted_velo(i,2), interpted_velo(i,3), :) = [interpted_velo(i,1),velo_cam(i,:)];
        end
        
        filled = bi_fillhole(velo_in_cam, 4);
        K = 0.125 * ones(3);
        s1 = conv2(filled(:,:,1), K, 'same');
        s2 = conv2(filled(:,:,2), K, 'same');
        s3 = conv2(filled(:,:,3), K, 'same');
        index1 = (s1 == 0);
        Xd = s1;
        Yd = s2;
        Zd = s3;
        projectionSize = size(Xd);
        [~, imgNormals, normalConf, NCompute] = ...
              		compute_local_planes(Xd, Yd, Zd, projectionSize);
        Ndash  = tvNormal(NCompute,1);
        Nx = Ndash(:,:,1);
        Ny = Ndash(:,:,2);
        Nz = Ndash(:,:,3);

        N = Nx.^2 + Ny.^2 + Nz.^2;
        N = N.^0.5;
        Nx = Nx ./ N; Ny = Ny ./ N; Nz = Nz ./ N;

        Nx(index1) = 0;
        Ny(index1) = 0;
        Nz(index1) = 0;
        n = cat(3, Nx, Ny, Nz);
        n_map = uint8((n/2+0.5)*255);
        % save to file
        img_name = [img_save_dir, '/', sequences(seq).name, '_', name, '.png'];
        normal_name = [normal_save_dir, '/', sequences(seq).name, '_', name, '.txt'];
        normal_map_name = [normal_map_save_dir, '/', sequences(seq).name, '_', name, '.png'];
        imwrite(img, img_name);
        imwrite(n_map, normal_map_name);
        fid = fopen(normal_name, 'wb');
        fwrite(fid, n, 'float');
        fclose(fid);
    end
end
end





