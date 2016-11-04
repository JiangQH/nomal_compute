function run_nyuNormalGenerate(base_dir, out_dir, output_depth)
    addpath('./matlab/');
    addpath('./toolbox/');
    addpath('./toolbox/tv_denoise');
    if nargin<1
        base_dir = '/home/qinhong/project/normal_recovery/data/indoor/nyu/raw';
    end
    if nargin<2
        out_dir = '/home/qinhong/project/normal_recovery/data/indoor/nyu/raw_train';
    end
    if nargin<3
        output_depth = true;
    end
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    % set the out dir of depth, normal and rgb
    out_normal_dir = [out_dir, '/normal'];
    if ~exist(out_normal_dir, 'dir')
        mkdir(out_normal_dir);
    end
    
    if output_depth
        out_depth_dir = [out_dir, '/depth'];
        if ~exist(out_depth_dir, 'dir')
            mkdir(out_depth_dir);
        end
    end
    
    out_rgb_dir = [out_dir, '/rgb'];
    if ~exist(out_rgb_dir, 'dir')
        mkdir(out_rgb_dir);
    end
    
    out_map_dir = [out_dir, '/normal_map'];
    if ~exist(out_map_dir, 'dir')
        mkdir(out_map_dir);
    end
    % get projectionMask
    [projectionMask, projectionSize] = get_projection_mask();
    % read the sequence folder
    sequences = dir(base_dir);
    parfor i = 3 : length(sequences)
        sequence_dir = [base_dir, '/', sequences(i).name];
        if ~exist(sequence_dir, 'dir')
            continue;
        end
        % read the data. but skip some of them
        framelist = get_synched_frames(sequence_dir);
        skip_ratio = 10;
        for j = 1 : skip_ratio : length(framelist)
            try
                rgb_file = [sequence_dir, '/', framelist(j).rawRgbFilename];
            catch
                break;
            end
            depth_file = [sequence_dir, '/', framelist(j).rawDepthFilename];
            
            % load and align depth and rgb
            imgDepth = imread(depth_file);
            imgDepth = swapbytes(imgDepth);
            imgRgb = imread(rgb_file);
            % project 
            [imgDepth2, imgRgb2] = project_depth_map(imgDepth, imgRgb);
            mask = (imgDepth2 ~= 0);
            
            % 1st method to compute normal
            % compute the 3d points
            point3d = rgb_plane2rgb_world(imgDepth2);
            point3d = point3d(projectionMask,:);
            X = point3d(:,1);
            Y = -point3d(:,2);
            Z = point3d(:,3);
            [imgPlanes, imgNormals, normalConf,NCompute] = ...
              		compute_local_planes(X, Y, Z, projectionSize);
                
            % valid depth data
            NMask = sum(NCompute.^2,3).^0.5 > 0.5;
        	depthValid = zeros(size(imgRgb2,1), size(imgRgb2,2));
        	depthValid(projectionMask) = NMask;
            
            % tv denoise the normal
            Ndash  = tvNormal(NCompute,1);
 	    	N1 = zeros(size(imgRgb2,1), size(imgRgb2,2));
            N1(projectionMask) = Ndash(:,:,1);

    	    N2 = zeros(size(imgRgb2,1), size(imgRgb2,2));
        	N2(projectionMask) = Ndash(:,:,2);

        	N3 = zeros(size(imgRgb2,1), size(imgRgb2,2));
        	N3(projectionMask) = Ndash(:,:,3);
            
        	Nn = (N1.^2 + N2.^2 + N3.^2).^0.5 + eps;
        	nx = N1./Nn; ny = N2./Nn; nz = N3./Nn;
            nx(~depthValid) = 0;
            ny(~depthValid) = 0;
            nz(~depthValid) = 0;
            normal = cat(3, nx, ny, nz);
           
            % save to file
            [~, name, ~] = fileparts(framelist(j).rawRgbFilename);
            normal_file = [out_normal_dir, '/', sequences(i).name, '_', name, '.txt'];
            rgb_file = [out_rgb_dir, '/', sequences(i).name, '_', name, '.png'];
            normal_map_file = [out_map_dir, '/', sequences(i).name, '_', name, '.png'];
            fid = fopen(normal_file, 'wb');
            fwrite(fid, normal, 'float');
            fclose(fid);
            imwrite(imgRgb2, rgb_file);
            n_map = uint8((normal/2+0.5)*255);
            imwrite(n_map, normal_map_file);
            if output_depth
                depth_file = [out_depth_dir, sequences(i).name, '_', name, '.txt'];
                fid = fopen(depth_file, 'wb');
                fwrite(fid, imgDepth2, 'float');
                fclose(fid);
            end
        end  
    end
    
end