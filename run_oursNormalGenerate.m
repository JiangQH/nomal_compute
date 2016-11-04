function run_oursNormalGenerate(base_dir)
    clear; clc;
    addpath('./matlab/');
    addpath('./toolbox/');
    addpath('./toolbox/tv_denoise');
    if nargin<1
        base_dir  = '/media/qinhong/Carrie/Jiang/Dataset/ours_rawdepth/cafe_0001a_rawDepth';
    end
    % set the img & depth & param file
    depth_dir = [base_dir, '/', 'DATA'];
    params = readParamsFromAct(fullfile(base_dir, 'result.act'));
    depth_files = dir(depth_dir);
    % do the transformation for every depth
    for i = 3 : length(depth_files)
        depth_file = fullfile(depth_dir, depth_files(i).name);
        [~, name, ~] = fileparts(depth_file);
        id = sprintf('%04d',str2num(name(7:length(name))));
        img_file = fullfile(base_dir, [id, '.jpg']);
        % read the depth and img
        img = imread(img_file);
        sizes = [size(img, 1), size(img, 2)];
        [depth, mask] = read_depth(depth_file, sizes);
        % fill depth colorization/filter the depth to make it smooth
        depth = fill_depth_colorization(double(img)./255.0, depth, 0.5);

%         bilateralSigmaSpatial = 24;
%         bilateralSigmaIntensity = 0.2;
%         depth = bfilter2(depth, bilateralSigmaSpatial*3, [bilateralSigmaSpatial,bilateralSigmaIntensity]);
        % do backproject to get the 3d points
        [Xd, Yd, Zd] = backproject(depth, params);
        Yd = -Yd;
        Xd = -Xd;
        % compute the normal
        project_size = sizes;
        [img_planes, img_normals, normal_conf, n_compute] = ...
              		compute_local_planes(Xd, Yd, Zd, project_size, true);
        % 2��tv-denoise
        NMask = sum(n_compute.^2,3).^0.5 > 0.5;
        depthValid = NMask;
        n_dash  = tvNormal(n_compute,1);
        Nx = n_dash(:,:,1);
        Ny = n_dash(:,:,2);
        Nz = n_dash(:,:,3);
        N = Nx.^2 + Ny.^2 + Nz.^2;
        N = N.^0.5;
        Nx = Nx ./ N; Ny = Ny ./ N; Nz = Nz ./ N;
        nx(~depthValid) = 0;
        ny(~depthValid) = 0;
        nz(~depthValid) = 0;
        n = cat(3, Nx, Ny, Nz);
        figure(1);
        imshow(img);
        figure(2);
        imshow(mat2gray(depth));
        figure(3);
        imshow(uint8((n/2+0.5)*255));
    end
end