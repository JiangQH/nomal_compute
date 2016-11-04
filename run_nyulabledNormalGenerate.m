function run_nyulabledNormalGenerate(in_dir, out_dir, output_depth)
    addpath('./matlab/');
    addpath('./toolbox/');
    addpath('./toolbox/tv_denoise');
    if nargin<3
        output_depth = false;
    end
    data_file = [in_dir, '/', 'nyu_depth_v2_labeled.mat'];
    split_file = [in_dir, '/', 'splits.mat'];
    rgb_out_dir = [out_dir, '/rgb'];
    normal_out_dir = [out_dir, '/normal'];
    depth_out_dir = [out_dir, '/depth'];
    load(data_file);
    load(split_file);
    % do it for train and test
    for cl = 1 : 2
    	if cl == 1
    		Ndxs = testNdxs;
    		rgb_out = [rgb_out_dir, '/test'];
    		normal_out = [normal_out_dir, '/test'];
    		depth_out = [depth_out_dir, '/test'];
    	end
    	if cl == 2
    		Ndxs = trainNdxs;
    		rgb_out = [rgb_out_dir, '/train'];
    		normal_out = [normal_out_dir, '/train'];
    		depth_out = [depth_out_dir, '/train'];
    	end
    	if ~exist(rgb_out, 'dir')
            mkdir(rgb_out);
        end
        if ~exist(normal_out, 'dir')
        	mkdir(normal_out);
        end
        if output_depth
        	if ~exist(depth_out, 'dir')
        		mkdir(depth_out);
        	end
        end
        for i = 1 : length(Ndxs)
            id = Ndxs(i);
            % load the image, and raw depth
            depth = rawDepths(:,:,id);
            img = images(:,:,:,id);
            % compute the 3d points
            point3d = rgb_plane2rgb_world(depth);
            X = point3d(:,1);
            Y = -point3d(:,2);
            Z = point3d(:,3);
            projectionSize = size(depth);
            [~,~,~,NCompute] = ...
                compute_local_planes(X, Y, Z, projectionSize);
            % valid depth data
            NMask = sum(NCompute.^2,3).^0.5 > 0.5;
            depthValid = NMask;
            % tv_denoise
            Ndash  = tvNormal(NCompute,1);
            N1 = Ndash(:,:,1);
            N2 = Ndash(:,:,2);
            N3 = Ndash(:,:,3);
            Nn = (N1.^2 + N2.^2 + N3.^2).^0.5 + eps;
            nx = N1./Nn; ny = N2./Nn; nz = N3./Nn;
            nx(~depthValid) = 0;
            ny(~depthValid) = 0;
            nz(~depthValid) = 0;
            normal = cat(3, nx, ny, nz);
            % save to file
            normal_file = [normal_out, '/', num2str(id), '.txt'];
            rgb_file = [rgb_out, '/', num2str(id), '.png'];
            fid = fopen(normal_file, 'wb');
            fwrite(fid, normal, 'float');
            fclose(fid);
            imwrite(img, rgb_file);
            if output_depth
            	depth_file = [depth_out, '/', num2str(id), '.txt'];
            	fid = fopen(depth_file, 'wb');
            	fwrite(fid, depth, 'float');
            	fclose(fid);
            end
        end
   	end
end